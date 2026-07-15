#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# SPDX-License-Identifier: AGPL-3.0-only

"""使用 YOLO + ByteTrack 进行多目标跟踪、轨迹预测，并向端点发送器提供
闭环反馈。

- 订阅解码帧（sensor_msgs/Image，由 h264_receiver 写入原始采集时间）。
- 运行 Ultralytics model.track()（ByteTrack）以获得稳定的跟踪 ID。
- 对近期轨迹进行最小二乘拟合，估计各目标在图像平面上的速度，并预测
  `horizon_s` 秒后的中心位置。
- 发布 DetectionArray（向后兼容）、TrackArray，以及带有轨迹和预测位置
  标注的图像。
- 反馈环路：跟踪到目标时，通过 /uav/stream_control 请求端点使用高帧率/
  码率配置；连续 `idle_timeout` 秒未跟踪到目标后请求空闲配置，从而在
  无事件发生时节省带宽。
"""

import collections

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from ultralytics import YOLO

from edge_yolo_ros.msg import Detection, DetectionArray, TrackArray, TrackedObject
from stream_msgs.msg import StreamControl


class TrackHistory(object):
    """单条轨迹近期的 (t, cx, cy) 样本。"""

    def __init__(self, maxlen=30):
        self.points = collections.deque(maxlen=maxlen)

    def add(self, stamp, cx, cy):
        self.points.append((stamp, cx, cy))

    def velocity(self):
        """根据近期样本以最小二乘法计算速度 (vx, vy)，单位为像素/秒。"""
        if len(self.points) < 3:
            return 0.0, 0.0
        t0 = self.points[0][0]
        ts = np.array([p[0] - t0 for p in self.points])
        xs = np.array([p[1] for p in self.points])
        ys = np.array([p[2] for p in self.points])
        span = ts[-1] - ts[0]
        if span < 1e-3:
            return 0.0, 0.0
        vx = float(np.polyfit(ts, xs, 1)[0])
        vy = float(np.polyfit(ts, ys, 1)[0])
        return vx, vy


class YoloTracker(object):
    def __init__(self):
        input_topic = rospy.get_param("~input_topic", "/uav/image_decoded")
        detections_topic = rospy.get_param("~detections_topic", "/yolo/detections")
        tracks_topic = rospy.get_param("~tracks_topic", "/yolo/tracks")
        annotated_topic = rospy.get_param(
            "~annotated_image_topic", "/yolo/annotated_image"
        )
        control_topic = rospy.get_param("~control_topic", "/uav/stream_control")
        weights = rospy.get_param("~weights")

        self.confidence = float(rospy.get_param("~confidence", 0.40))
        self.iou_threshold = float(rospy.get_param("~iou_threshold", 0.45))
        self.image_size = int(rospy.get_param("~image_size", 640))
        self.device = rospy.get_param("~device", "cpu")
        self.tracker_config = rospy.get_param("~tracker", "bytetrack.yaml")
        self.horizon_s = float(rospy.get_param("~prediction_horizon_s", 0.5))
        self.publish_annotated = bool(
            rospy.get_param("~publish_annotated_image", True)
        )

        # 反馈配置档位。
        self.active_fps = float(rospy.get_param("~active_fps", 15.0))
        self.active_bitrate = int(rospy.get_param("~active_bitrate_kbps", 1800))
        self.idle_fps = float(rospy.get_param("~idle_fps", 3.0))
        self.idle_bitrate = int(rospy.get_param("~idle_bitrate_kbps", 400))
        self.idle_timeout = float(rospy.get_param("~idle_timeout", 3.0))
        self.control_refresh = float(rospy.get_param("~control_refresh", 2.0))

        rospy.loginfo("Loading YOLO weights: %s", weights)
        self.model = YOLO(weights)

        self.histories = {}
        self.last_seen = {}
        self.last_track_time = rospy.Time(0)
        self.current_profile = None
        self.last_control_publish = rospy.Time(0)

        self.detections_publisher = rospy.Publisher(
            detections_topic, DetectionArray, queue_size=1
        )
        self.tracks_publisher = rospy.Publisher(tracks_topic, TrackArray, queue_size=1)
        self.image_publisher = rospy.Publisher(annotated_topic, Image, queue_size=1)
        self.control_publisher = rospy.Publisher(
            control_topic, StreamControl, queue_size=1
        )
        self.subscriber = rospy.Subscriber(
            input_topic, Image, self.image_callback,
            queue_size=1, buff_size=2 ** 24, tcp_nodelay=True,
        )
        self.control_timer = rospy.Timer(
            rospy.Duration(self.control_refresh), self.refresh_control
        )

        rospy.loginfo(
            "YOLO tracker: %s -> %s / %s (tracker=%s, horizon=%.1fs)",
            input_topic, detections_topic, tracks_topic,
            self.tracker_config, self.horizon_s,
        )

    # ------------------------------------------------------------------ #
    # 推理与跟踪
    # ------------------------------------------------------------------ #
    def image_callback(self, message):
        frame = np.frombuffer(message.data, dtype=np.uint8)
        try:
            frame = frame.reshape((message.height, message.width, 3))
        except ValueError:
            rospy.logwarn_throttle(5.0, "Unexpected image buffer size")
            return

        try:
            result = self.model.track(
                source=frame,
                persist=True,
                conf=self.confidence,
                iou=self.iou_threshold,
                imgsz=self.image_size,
                device=self.device,
                tracker=self.tracker_config,
                verbose=False,
            )[0]
        except Exception as error:
            rospy.logerr_throttle(5.0, "YOLO tracking failed: %s", error)
            return

        stamp_sec = message.header.stamp.to_sec()
        detections_msg, tracks_msg, drawn = self.build_messages(
            message, frame, result, stamp_sec
        )

        self.detections_publisher.publish(detections_msg)
        self.tracks_publisher.publish(tracks_msg)
        if self.publish_annotated:
            self.image_publisher.publish(self.to_image_message(message, drawn))

        if tracks_msg.tracks:
            self.last_track_time = rospy.Time.now()
        self.update_control(len(tracks_msg.tracks))

        rospy.loginfo_throttle(
            5.0, "Tracking %d objects, %.1f ms inference",
            len(tracks_msg.tracks), detections_msg.inference_ms,
        )

    def build_messages(self, source, frame, result, stamp_sec):
        detections = DetectionArray()
        detections.header = source.header
        detections.image_height = frame.shape[0]
        detections.image_width = frame.shape[1]
        detections.inference_ms = float(result.speed.get("inference", 0.0))

        tracks = TrackArray()
        tracks.header = source.header
        tracks.image_height = frame.shape[0]
        tracks.image_width = frame.shape[1]
        tracks.horizon_s = self.horizon_s

        drawn = np.ascontiguousarray(result.plot()) if self.publish_annotated \
            else frame
        names = result.names
        active_ids = set()

        if result.boxes is not None:
            for box in result.boxes:
                class_id = int(box.cls[0].item())
                coords = box.xyxy[0].detach().cpu().tolist()
                conf = float(box.conf[0].item())
                if isinstance(names, dict):
                    class_name = str(names.get(class_id, class_id))
                else:
                    class_name = str(names[class_id])

                detection = Detection()
                detection.class_id = class_id
                detection.class_name = class_name
                detection.confidence = conf
                detection.xmin, detection.ymin = coords[0], coords[1]
                detection.xmax, detection.ymax = coords[2], coords[3]
                detections.detections.append(detection)

                if box.id is None:
                    continue
                track_id = int(box.id[0].item())
                active_ids.add(track_id)
                cx = 0.5 * (coords[0] + coords[2])
                cy = 0.5 * (coords[1] + coords[3])

                history = self.histories.setdefault(track_id, TrackHistory())
                history.add(stamp_sec, cx, cy)
                self.last_seen[track_id] = rospy.Time.now()
                vx, vy = history.velocity()

                track = TrackedObject()
                track.track_id = track_id
                track.class_id = class_id
                track.class_name = class_name
                track.confidence = conf
                track.xmin, track.ymin = coords[0], coords[1]
                track.xmax, track.ymax = coords[2], coords[3]
                track.vx, track.vy = vx, vy
                track.pred_x = cx + vx * self.horizon_s
                track.pred_y = cy + vy * self.horizon_s
                tracks.tracks.append(track)

                if self.publish_annotated:
                    self.draw_track(drawn, history, track)

        self.prune_histories(active_ids)
        return detections, tracks, drawn

    def draw_track(self, image, history, track):
        points = [(int(p[1]), int(p[2])) for p in history.points]
        if len(points) >= 2:
            cv2.polylines(
                image, [np.array(points, dtype=np.int32)], False, (255, 200, 0), 2
            )
        current = points[-1] if points else None
        predicted = (int(track.pred_x), int(track.pred_y))
        if current is not None and (abs(track.vx) + abs(track.vy)) > 1.0:
            cv2.arrowedLine(image, current, predicted, (0, 0, 255), 2, tipLength=0.3)
            cv2.circle(image, predicted, 4, (0, 0, 255), -1)

    def prune_histories(self, active_ids):
        now = rospy.Time.now()
        stale = [
            track_id for track_id, seen in self.last_seen.items()
            if track_id not in active_ids and (now - seen).to_sec() > 5.0
        ]
        for track_id in stale:
            self.histories.pop(track_id, None)
            self.last_seen.pop(track_id, None)

    def to_image_message(self, source, frame):
        message = Image()
        message.header = source.header
        message.height = frame.shape[0]
        message.width = frame.shape[1]
        message.encoding = "bgr8"
        message.is_bigendian = 0
        message.step = frame.shape[1] * 3
        message.data = frame.tobytes()
        return message

    # ------------------------------------------------------------------ #
    # 向端点提供闭环反馈
    # ------------------------------------------------------------------ #
    def desired_profile(self, num_tracks):
        if num_tracks > 0:
            return "active"
        idle_for = (rospy.Time.now() - self.last_track_time).to_sec()
        return "idle" if idle_for >= self.idle_timeout else "active"

    def update_control(self, num_tracks):
        profile = self.desired_profile(num_tracks)
        if profile != self.current_profile:
            self.current_profile = profile
            self.publish_control(
                "tracking %d objects" % num_tracks if profile == "active"
                else "no targets for %.1fs" % self.idle_timeout
            )

    def refresh_control(self, _event):
        # 定期重新发布，使重启后的发送器也能获取当前配置档位。
        if self.current_profile is not None:
            self.publish_control("periodic refresh")

    def publish_control(self, reason):
        control = StreamControl()
        control.header.stamp = rospy.Time.now()
        if self.current_profile == "active":
            control.target_fps = self.active_fps
            control.target_bitrate_kbps = self.active_bitrate
        else:
            control.target_fps = self.idle_fps
            control.target_bitrate_kbps = self.idle_bitrate
        control.reason = "%s profile: %s" % (self.current_profile, reason)
        self.control_publisher.publish(control)
        self.last_control_publish = rospy.Time.now()


def main():
    rospy.init_node("yolo_tracker")
    YoloTracker()
    rospy.spin()


if __name__ == "__main__":
    main()
