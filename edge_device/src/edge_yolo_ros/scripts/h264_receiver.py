#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# SPDX-License-Identifier: AGPL-3.0-only

"""用于带外视频传输的 RTP/H.264 接收器。

接收端点设备 h264_sender 发送的 UDP RTP 视频流，使用 GStreamer 解码，
并在普通 ROS 话题上将解码帧重新发布为 sensor_msgs/Image，使所有下游
ROS 节点（YOLO 跟踪器、可视化工具、rosbag）无需修改即可继续工作。

解码帧按照先进先出顺序与 /uav/stream_meta 消息配对，使重新发布的
Image 携带端点相机的原始采集时间戳，从而能够测量端到端延迟。
"""

import collections
import threading

import rospy
from sensor_msgs.msg import Image
from stream_msgs.msg import StreamMeta

import gi
gi.require_version("Gst", "1.0")
from gi.repository import GLib, Gst


class H264Receiver(object):
    def __init__(self):
        Gst.init(None)

        self.port = int(rospy.get_param("~port", 5600))
        self.jitter_latency_ms = int(rospy.get_param("~jitter_latency_ms", 50))
        meta_topic = rospy.get_param("~meta_topic", "/uav/stream_meta")
        output_topic = rospy.get_param("~output_topic", "/uav/image_decoded")
        # 超过此时限的元数据将被视为过期并丢弃。
        self.meta_max_age = float(rospy.get_param("~meta_max_age", 2.0))

        self.meta_lock = threading.Lock()
        self.meta_queue = collections.deque(maxlen=64)
        self.unmatched_frames = 0
        self.frames_out = 0

        self.publisher = rospy.Publisher(output_topic, Image, queue_size=1)
        self.meta_subscriber = rospy.Subscriber(
            meta_topic, StreamMeta, self.meta_callback, queue_size=32
        )

        description = (
            "udpsrc port=%d caps=\"application/x-rtp,media=video,"
            "encoding-name=H264,payload=96,clock-rate=90000\" ! "
            "rtpjitterbuffer latency=%d ! rtph264depay ! h264parse ! "
            "avdec_h264 ! videoconvert ! video/x-raw,format=BGR ! "
            "appsink name=sink emit-signals=true sync=false drop=true max-buffers=2"
            % (self.port, self.jitter_latency_ms)
        )
        rospy.loginfo("GStreamer pipeline: %s", description)

        self.pipeline = Gst.parse_launch(description)
        self.appsink = self.pipeline.get_by_name("sink")
        self.appsink.connect("new-sample", self.on_new_sample)
        self.pipeline.set_state(Gst.State.PLAYING)

        self.loop = GLib.MainLoop()
        self.loop_thread = threading.Thread(target=self.loop.run)
        self.loop_thread.daemon = True
        self.loop_thread.start()
        rospy.on_shutdown(self.shutdown)

        rospy.loginfo(
            "H264 receiver: udp:%d -> %s (meta from %s)",
            self.port, output_topic, meta_topic,
        )

    def meta_callback(self, message):
        with self.meta_lock:
            self.meta_queue.append(message)

    def pop_matching_meta(self):
        """按照先进先出顺序配对，并清理过期条目。"""
        now = rospy.Time.now()
        with self.meta_lock:
            while self.meta_queue:
                meta = self.meta_queue.popleft()
                if (now - meta.sent_stamp).to_sec() <= self.meta_max_age:
                    return meta
        return None

    def on_new_sample(self, sink):
        sample = sink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.OK

        caps = sample.get_caps().get_structure(0)
        width = caps.get_value("width")
        height = caps.get_value("height")

        buffer = sample.get_buffer()
        ok, map_info = buffer.map(Gst.MapFlags.READ)
        if not ok:
            return Gst.FlowReturn.OK
        try:
            data = bytes(map_info.data)
        finally:
            buffer.unmap(map_info)

        expected = width * height * 3
        if len(data) < expected:
            rospy.logwarn_throttle(5.0, "Decoded buffer smaller than expected")
            return Gst.FlowReturn.OK

        meta = self.pop_matching_meta()

        message = Image()
        if meta is not None:
            message.header.stamp = meta.header.stamp  # 原始采集时间
            message.header.frame_id = meta.header.frame_id
            message.header.seq = meta.seq
        else:
            self.unmatched_frames += 1
            message.header.stamp = rospy.Time.now()
            message.header.frame_id = "uav_camera_unsynced"
        message.height = height
        message.width = width
        message.encoding = "bgr8"
        message.is_bigendian = 0
        message.step = width * 3
        message.data = data[:expected]

        self.publisher.publish(message)
        self.frames_out += 1
        if self.frames_out % 300 == 0:
            rospy.loginfo(
                "H264 receiver: %d frames decoded (%d without metadata match)",
                self.frames_out, self.unmatched_frames,
            )
        return Gst.FlowReturn.OK

    def shutdown(self):
        try:
            self.pipeline.set_state(Gst.State.NULL)
            self.loop.quit()
        except Exception:
            pass


def main():
    rospy.init_node("h264_receiver")
    H264Receiver()
    rospy.spin()


if __name__ == "__main__":
    main()
