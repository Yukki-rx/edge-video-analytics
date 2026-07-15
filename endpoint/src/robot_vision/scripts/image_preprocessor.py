#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Resize, throttle and JPEG-compress usb_cam images for edge inference."""

from __future__ import print_function

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image


class ImagePreprocessor(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.last_publish_time = rospy.Time(0)

        input_topic = rospy.get_param("~input_topic", "/usb_cam/image_raw")
        output_topic = rospy.get_param(
            "~output_topic", "/uav/image_preprocessed/compressed"
        )
        self.output_fps = float(rospy.get_param("~output_fps", 10.0))
        self.output_width = int(rospy.get_param("~output_width", 416))
        self.output_height = int(rospy.get_param("~output_height", 312))
        self.jpeg_quality = int(rospy.get_param("~jpeg_quality", 70))

        if self.output_fps <= 0:
            raise ValueError("~output_fps must be greater than zero")
        if self.output_width <= 0 or self.output_height <= 0:
            raise ValueError("output image width and height must be greater than zero")
        if not 1 <= self.jpeg_quality <= 100:
            raise ValueError("~jpeg_quality must be between 1 and 100")

        self.publisher = rospy.Publisher(
            output_topic, CompressedImage, queue_size=1
        )
        self.subscriber = rospy.Subscriber(
            input_topic,
            Image,
            self.image_callback,
            queue_size=1,
            buff_size=2 ** 24,
        )

        rospy.loginfo("Image preprocessor input: %s", input_topic)
        rospy.loginfo("Image preprocessor output: %s", output_topic)
        rospy.loginfo(
            "Output: %dx%d, %.1f FPS, JPEG quality %d",
            self.output_width,
            self.output_height,
            self.output_fps,
            self.jpeg_quality,
        )

    def image_callback(self, message):
        now = rospy.Time.now()
        interval = 1.0 / self.output_fps
        if (now - self.last_publish_time).to_sec() < interval:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(message, "bgr8")
        except CvBridgeError as error:
            rospy.logerr_throttle(5.0, "cv_bridge conversion failed: %s", error)
            return

        frame = cv2.resize(
            frame,
            (self.output_width, self.output_height),
            interpolation=cv2.INTER_AREA,
        )
        success, encoded = cv2.imencode(
            ".jpg",
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )
        if not success:
            rospy.logwarn_throttle(5.0, "JPEG encoding failed")
            return

        output = CompressedImage()
        output.header = message.header
        output.format = "jpeg"
        # tostring() works with both the Python 2 and Python 3 versions
        # commonly found on ROS Melodic/Noetic systems.
        output.data = encoded.tostring()
        self.publisher.publish(output)
        self.last_publish_time = now


def main():
    rospy.init_node("image_preprocessor")
    ImagePreprocessor()
    rospy.loginfo("Image preprocessor started")
    rospy.spin()


if __name__ == "__main__":
    main()
