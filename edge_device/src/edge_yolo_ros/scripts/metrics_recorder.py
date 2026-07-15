#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# SPDX-License-Identifier: AGPL-3.0-only

"""记录 baseline 链路的传输、检测延迟和推理指标。"""

import csv
import json
import os
import threading
import time

import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage

from edge_yolo_ros.msg import DetectionArray


def stats_block(values):
    if not values:
        return {"count": 0}
    array = np.asarray(values, dtype=np.float64)
    return {
        "count": int(array.size),
        "mean": round(float(np.mean(array)), 2),
        "median": round(float(np.median(array)), 2),
        "p95": round(float(np.percentile(array, 95)), 2),
        "max": round(float(np.max(array)), 2),
    }


class MetricsRecorder(object):
    def __init__(self):
        self.results_dir = os.path.expanduser(
            rospy.get_param("~results_dir", "~/edge_yolo_ros/results")
        )
        transport_topic = rospy.get_param(
            "~transport_topic", "/uav/image_preprocessed/compressed"
        )
        detections_topic = rospy.get_param(
            "~detections_topic", "/yolo/detections"
        )

        self.lock = threading.Lock()
        self.start_wall = time.time()
        self.transport_rows = []
        self.detection_rows = []

        rospy.Subscriber(
            transport_topic,
            CompressedImage,
            self.transport_callback,
            queue_size=8,
        )
        rospy.Subscriber(
            detections_topic,
            DetectionArray,
            self.detections_callback,
            queue_size=8,
        )
        rospy.on_shutdown(self.write_results)
        rospy.loginfo("Baseline metrics recorder started: %s", self.results_dir)

    @staticmethod
    def latency_ms(stamp):
        if stamp == rospy.Time(0):
            return None
        value = (rospy.Time.now() - stamp).to_sec() * 1000.0
        if value < -1000.0 or value > 60000.0:
            return None
        return value

    def transport_callback(self, message):
        with self.lock:
            self.transport_rows.append(
                (time.time(), len(message.data), self.latency_ms(message.header.stamp))
            )

    def detections_callback(self, message):
        with self.lock:
            self.detection_rows.append(
                (
                    time.time(),
                    self.latency_ms(message.header.stamp),
                    float(message.inference_ms),
                    len(message.detections),
                )
            )

    def build_summary(self):
        duration = max(time.time() - self.start_wall, 1e-6)
        with self.lock:
            transport = list(self.transport_rows)
            detections = list(self.detection_rows)

        total_bytes = sum(row[1] for row in transport)
        transport_latencies = [row[2] for row in transport if row[2] is not None]
        end_to_end_latencies = [
            row[1] for row in detections if row[1] is not None
        ]
        inference_times = [row[2] for row in detections]
        detection_counts = [row[3] for row in detections]

        summary = {
            "mode": "baseline",
            "label": "JPEG over ROS + YOLO",
            "started_at": time.strftime(
                "%Y-%m-%d %H:%M:%S", time.localtime(self.start_wall)
            ),
            "duration_s": round(duration, 1),
            "transport": {
                "frames": len(transport),
                "avg_fps": round(len(transport) / duration, 2),
                "total_mb": round(total_bytes / 1e6, 3),
                "avg_kbps": round(total_bytes * 8.0 / duration / 1000.0, 1),
                "avg_bytes_per_frame": (
                    int(total_bytes / len(transport)) if transport else 0
                ),
            },
            "latency_ms": {
                "transport": stats_block(transport_latencies),
                "end_to_end_detection": stats_block(end_to_end_latencies),
            },
            "inference_ms": stats_block(inference_times),
            "detections_per_frame": stats_block(detection_counts),
            "clock_sync_note": (
                "Cross-device latencies assume synchronized endpoint and edge clocks."
            ),
        }
        return summary, transport, detections

    def write_results(self):
        summary, transport, detections = self.build_summary()
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        run_dir = os.path.join(self.results_dir, "baseline_%s" % timestamp)
        os.makedirs(run_dir, exist_ok=True)

        with open(
            os.path.join(run_dir, "summary.json"), "w", encoding="utf-8"
        ) as handle:
            json.dump(summary, handle, indent=2, ensure_ascii=False)

        with open(
            os.path.join(run_dir, "frames.csv"), "w", newline="", encoding="utf-8"
        ) as handle:
            writer = csv.writer(handle)
            writer.writerow(["type", "wall_time", "value_1", "value_2", "value_3"])
            for row in transport:
                writer.writerow(
                    [
                        "transport",
                        "%.3f" % row[0],
                        row[1],
                        "" if row[2] is None else "%.2f" % row[2],
                        "",
                    ]
                )
            for row in detections:
                writer.writerow(
                    [
                        "detection",
                        "%.3f" % row[0],
                        "" if row[1] is None else "%.2f" % row[1],
                        "%.2f" % row[2],
                        row[3],
                    ]
                )

        print("[metrics_recorder] results written to %s" % run_dir)


def main():
    rospy.init_node("metrics_recorder")
    MetricsRecorder()
    rospy.spin()


if __name__ == "__main__":
    main()
