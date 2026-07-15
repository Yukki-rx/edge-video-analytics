#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Subscribe to a ROS CompressedImage topic and run Ultralytics YOLO."""

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage, Image
from ultralytics import YOLO

from edge_yolo_ros.msg import Detection, DetectionArray


class YoloDetector(object):
    def __init__(self):
        self.input_topic = rospy.get_param(
            "~input_topic", "/uav/image_preprocessed/compressed"
        )
        detections_topic = rospy.get_param(
            "~detections_topic", "/yolo/detections"
        )
        annotated_image_topic = rospy.get_param(
            "~annotated_image_topic", "/yolo/annotated_image"
        )
        weights = rospy.get_param("~weights")

        self.confidence = float(rospy.get_param("~confidence", 0.40))
        self.iou_threshold = float(rospy.get_param("~iou_threshold", 0.45))
        self.image_size = int(rospy.get_param("~image_size", 640))
        self.max_detections = int(rospy.get_param("~max_detections", 100))
        self.device = rospy.get_param("~device", "cpu")
        self.publish_annotated_image = bool(
            rospy.get_param("~publish_annotated_image", True)
        )

        rospy.loginfo("Loading YOLO weights: %s", weights)
        rospy.loginfo("Inference device: %s", self.device)
        self.model = YOLO(weights)

        self.detections_publisher = rospy.Publisher(
            detections_topic, DetectionArray, queue_size=1
        )
        self.image_publisher = rospy.Publisher(
            annotated_image_topic, Image, queue_size=1
        )
        self.image_subscriber = rospy.Subscriber(
            self.input_topic,
            CompressedImage,
            self.image_callback,
            queue_size=1,
            buff_size=2 ** 24,
            tcp_nodelay=True,
        )

        rospy.loginfo("YOLO input: %s", self.input_topic)
        rospy.loginfo("YOLO detections: %s", detections_topic)
        rospy.loginfo("YOLO annotated image: %s", annotated_image_topic)

    def image_callback(self, message):
        encoded = np.frombuffer(message.data, dtype=np.uint8)
        frame = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if frame is None:
            rospy.logwarn_throttle(5.0, "Failed to decode compressed image")
            return

        try:
            result = self.model.predict(
                source=frame,
                conf=self.confidence,
                iou=self.iou_threshold,
                imgsz=self.image_size,
                max_det=self.max_detections,
                device=self.device,
                verbose=False,
            )[0]
        except Exception as error:  # Keep the ROS node alive after one bad frame.
            rospy.logerr_throttle(5.0, "YOLO inference failed: %s", error)
            return

        detections_message = self.create_detections_message(message, frame, result)
        self.detections_publisher.publish(detections_message)

        if self.publish_annotated_image:
            annotated = np.ascontiguousarray(result.plot())
            self.image_publisher.publish(
                self.create_image_message(message, annotated)
            )

        rospy.loginfo_throttle(
            5.0,
            "YOLO running: %d objects, %.1f ms inference",
            len(detections_message.detections),
            detections_message.inference_ms,
        )

    def create_detections_message(self, source_message, frame, result):
        output = DetectionArray()
        output.header = source_message.header
        output.image_height = frame.shape[0]
        output.image_width = frame.shape[1]
        output.inference_ms = float(result.speed.get("inference", 0.0))

        names = result.names
        if result.boxes is None:
            return output

        for box in result.boxes:
            class_id = int(box.cls[0].item())
            coordinates = box.xyxy[0].detach().cpu().tolist()

            detection = Detection()
            detection.class_id = class_id
            if isinstance(names, dict):
                detection.class_name = str(names.get(class_id, class_id))
            else:
                detection.class_name = str(names[class_id])
            detection.confidence = float(box.conf[0].item())
            detection.xmin = float(coordinates[0])
            detection.ymin = float(coordinates[1])
            detection.xmax = float(coordinates[2])
            detection.ymax = float(coordinates[3])
            output.detections.append(detection)

        return output

    @staticmethod
    def create_image_message(source_message, image):
        output = Image()
        output.header = source_message.header
        output.height = image.shape[0]
        output.width = image.shape[1]
        output.encoding = "bgr8"
        output.is_bigendian = 0
        output.step = image.shape[1] * 3
        output.data = image.tobytes()
        return output


def main():
    rospy.init_node("yolo_detector")
    YoloDetector()
    rospy.loginfo("YOLO detector started")
    rospy.spin()


if __name__ == "__main__":
    main()
