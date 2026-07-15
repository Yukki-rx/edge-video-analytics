#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""内容感知运动门控。

订阅原始相机视频流，仅转发包含运动的帧（在缩小后的灰度图像上使用
MOG2 背景减除）。保活间隔可保证最低转发速率，避免 H.264 视频流和
边缘端跟踪器完全断流。
"""

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from stream_msgs.msg import GateStats


class MotionGate(object):
    def __init__(self):
        input_topic = rospy.get_param("~input_topic", "/usb_cam/image_raw")
        output_topic = rospy.get_param("~output_topic", "/uav/image_gated")
        stats_topic = rospy.get_param("~stats_topic", "/uav/gate_stats")

        # 当前景像素占比超过此值时，将该帧视为存在运动。
        self.motion_threshold = float(rospy.get_param("~motion_threshold", 0.005))
        # 每隔 keepalive_interval 秒至少转发一帧。
        self.keepalive_interval = float(rospy.get_param("~keepalive_interval", 2.0))
        # 内部运动分析图像所使用的宽度。
        self.analysis_width = int(rospy.get_param("~analysis_width", 160))
        # 在最初的预热时段内转发所有帧（用于背景学习）。
        self.warmup_duration = float(rospy.get_param("~warmup_duration", 3.0))

        self.bridge = CvBridge()
        self.subtractor = cv2.createBackgroundSubtractorMOG2(
            history=200, varThreshold=25, detectShadows=False
        )
        self.start_time = None
        self.last_pass_time = rospy.Time(0)
        self.frames_in = 0
        self.frames_passed = 0
        self.last_motion_ratio = 0.0

        self.publisher = rospy.Publisher(output_topic, Image, queue_size=1)
        self.stats_publisher = rospy.Publisher(stats_topic, GateStats, queue_size=1)
        self.subscriber = rospy.Subscriber(
            input_topic, Image, self.image_callback, queue_size=1, buff_size=2 ** 24
        )
        self.stats_timer = rospy.Timer(rospy.Duration(1.0), self.publish_stats)

        rospy.loginfo(
            "Motion gate: %s -> %s (threshold %.4f, keepalive %.1fs)",
            input_topic, output_topic, self.motion_threshold, self.keepalive_interval,
        )

    def compute_motion_ratio(self, message):
        """在缩小的灰度副本上运行 MOG2，并返回前景像素占比。"""
        gray = self.bridge.imgmsg_to_cv2(message, desired_encoding="mono8")
        scale = float(self.analysis_width) / gray.shape[1]
        small = cv2.resize(
            gray,
            (self.analysis_width, max(1, int(round(gray.shape[0] * scale)))),
            interpolation=cv2.INTER_AREA,
        )
        mask = self.subtractor.apply(small)
        return float(np.count_nonzero(mask)) / float(mask.size)

    def image_callback(self, message):
        now = rospy.Time.now()
        if self.start_time is None:
            self.start_time = now
        self.frames_in += 1

        try:
            motion_ratio = self.compute_motion_ratio(message)
        except (CvBridgeError, cv2.error) as error:
            rospy.logwarn_throttle(5.0, "Motion analysis failed: %s", error)
            motion_ratio = 1.0  # 故障时保持开放：发生错误时绝不丢帧
        self.last_motion_ratio = motion_ratio

        warming_up = (now - self.start_time).to_sec() < self.warmup_duration
        keepalive_due = (now - self.last_pass_time).to_sec() >= self.keepalive_interval
        has_motion = motion_ratio >= self.motion_threshold

        if warming_up or has_motion or keepalive_due:
            self.publisher.publish(message)
            self.frames_passed += 1
            self.last_pass_time = now

    def publish_stats(self, _event):
        stats = GateStats()
        stats.header.stamp = rospy.Time.now()
        stats.frames_in = self.frames_in
        stats.frames_passed = self.frames_passed
        stats.pass_ratio = (
            float(self.frames_passed) / self.frames_in if self.frames_in else 0.0
        )
        stats.motion_ratio = self.last_motion_ratio
        self.stats_publisher.publish(stats)


def main():
    rospy.init_node("motion_gate")
    MotionGate()
    rospy.spin()


if __name__ == "__main__":
    main()
