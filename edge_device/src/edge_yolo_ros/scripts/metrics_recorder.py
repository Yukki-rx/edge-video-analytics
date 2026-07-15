#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# SPDX-License-Identifier: AGPL-3.0-only

"""单次运行指标记录器及基线方案与改进方案的对比报告生成器。

在边缘设备的两种模式下均会运行：

  baseline：通过 ROS 传输 JPEG（/uav/image_preprocessed/compressed）+ 纯 YOLO
  improved：运动门控 + H.264 RTP/UDP + ByteTrack + 反馈环路

逐帧采集传输字节数、传输延迟（采集 -> 边缘端）、端到端延迟
（采集 -> 检测结果发布）、推理耗时以及检测/跟踪数量。关闭时
（Ctrl-C）会写入：

  <results_dir>/<mode>_<timestamp>/summary.json
  <results_dir>/<mode>_<timestamp>/frames.csv

如果存在另一种模式的运行结果，还会生成对比报告：

  <results_dir>/comparison_<timestamp>.md (+ .json)

该报告会比较两种模式各自最近一次的运行结果。

注意：跨设备延迟测量要求两台设备的时钟保持同步（chrony/NTP）。
摘要中会记录这一限制条件。
"""

import csv
import glob
import json
import os
import threading
import time

import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage, Image

from edge_yolo_ros.msg import DetectionArray

try:
    from edge_yolo_ros.msg import TrackArray
except ImportError:  # 不含跟踪消息的旧版本
    TrackArray = None

try:
    from stream_msgs.msg import GateStats, StreamControl, StreamMeta
except ImportError:  # 仅支持基线模式的工作空间
    GateStats = StreamControl = StreamMeta = None


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
        self.mode = rospy.get_param("~mode", "improved")
        if self.mode not in ("baseline", "improved"):
            raise ValueError("~mode must be 'baseline' or 'improved'")
        self.results_dir = os.path.expanduser(
            rospy.get_param("~results_dir", "~/edge_yolo_ros/results")
        )
        self.generate_comparison = bool(rospy.get_param("~generate_comparison", True))

        self.lock = threading.Lock()
        self.start_wall = time.time()
        self.start_ros = rospy.Time.now()

        # 逐帧传输记录：(recv_time, bytes, transport_latency_ms)
        self.transport_rows = []
        # 逐次检测记录：(recv_time, e2e_latency_ms, inference_ms, num_det)
        self.detection_rows = []
        self.track_counts = []
        self.control_events = []
        self.gate_last = None
        self.stream_settings = []  # (fps_setting, bitrate_kbps)
        # 解码帧延迟记录（仅改进模式）：(recv_time, latency_ms)
        self.decode_rows = []

        if self.mode == "baseline":
            transport_topic = rospy.get_param(
                "~baseline_transport_topic", "/uav/image_preprocessed/compressed"
            )
            rospy.Subscriber(
                transport_topic, CompressedImage,
                self.baseline_transport_callback, queue_size=8,
            )
        else:
            if StreamMeta is None:
                raise RuntimeError(
                    "stream_msgs not built - required for improved mode"
                )
            rospy.Subscriber(
                rospy.get_param("~meta_topic", "/uav/stream_meta"),
                StreamMeta, self.meta_callback, queue_size=32,
            )
            rospy.Subscriber(
                rospy.get_param("~decoded_topic", "/uav/image_decoded"),
                Image, self.decoded_callback, queue_size=4,
            )
            rospy.Subscriber(
                rospy.get_param("~control_topic", "/uav/stream_control"),
                StreamControl, self.control_callback, queue_size=8,
            )
            rospy.Subscriber(
                rospy.get_param("~gate_stats_topic", "/uav/gate_stats"),
                GateStats, self.gate_callback, queue_size=4,
            )
            if TrackArray is not None:
                rospy.Subscriber(
                    rospy.get_param("~tracks_topic", "/yolo/tracks"),
                    TrackArray, self.tracks_callback, queue_size=4,
                )

        rospy.Subscriber(
            rospy.get_param("~detections_topic", "/yolo/detections"),
            DetectionArray, self.detections_callback, queue_size=8,
        )

        rospy.on_shutdown(self.write_results)
        rospy.loginfo(
            "Metrics recorder started (mode=%s, results_dir=%s)",
            self.mode, self.results_dir,
        )

    # ------------------------------------------------------------------ #
    # 回调函数
    # ------------------------------------------------------------------ #
    @staticmethod
    def latency_ms(stamp):
        if stamp == rospy.Time(0):
            return None
        value = (rospy.Time.now() - stamp).to_sec() * 1000.0
        # 丢弃因时钟偏差产生的严重负值或明显异常值。
        if value < -1000.0 or value > 60000.0:
            return None
        return value

    def baseline_transport_callback(self, message):
        latency = self.latency_ms(message.header.stamp)
        with self.lock:
            self.transport_rows.append(
                (time.time(), len(message.data), latency)
            )

    def meta_callback(self, message):
        capture_to_sent = (message.sent_stamp - message.header.stamp).to_sec() * 1000.0
        with self.lock:
            self.transport_rows.append(
                (time.time(), int(message.bytes), max(capture_to_sent, 0.0))
            )
            self.stream_settings.append(
                (float(message.fps_setting), int(message.bitrate_kbps))
            )

    def decoded_callback(self, message):
        latency = self.latency_ms(message.header.stamp)
        if latency is None:
            return
        with self.lock:
            self.decode_rows.append((time.time(), latency))

    def detections_callback(self, message):
        latency = self.latency_ms(message.header.stamp)
        with self.lock:
            self.detection_rows.append(
                (
                    time.time(),
                    latency,
                    float(message.inference_ms),
                    len(message.detections),
                )
            )

    def tracks_callback(self, message):
        with self.lock:
            self.track_counts.append(len(message.tracks))

    def control_callback(self, message):
        with self.lock:
            self.control_events.append(
                {
                    "time": time.time(),
                    "target_fps": float(message.target_fps),
                    "target_bitrate_kbps": int(message.target_bitrate_kbps),
                    "reason": message.reason,
                }
            )

    def gate_callback(self, message):
        with self.lock:
            self.gate_last = {
                "frames_in": int(message.frames_in),
                "frames_passed": int(message.frames_passed),
                "pass_ratio": round(float(message.pass_ratio), 4),
            }

    # ------------------------------------------------------------------ #
    # 结果写入
    # ------------------------------------------------------------------ #
    def build_summary(self):
        duration = max(time.time() - self.start_wall, 1e-6)
        with self.lock:
            transport = list(self.transport_rows)
            detections = list(self.detection_rows)
            decode = list(self.decode_rows)
            tracks = list(self.track_counts)
            controls = list(self.control_events)
            gate = dict(self.gate_last) if self.gate_last else None
            settings = list(self.stream_settings)

        total_bytes = sum(row[1] for row in transport)
        transport_latencies = [row[2] for row in transport if row[2] is not None]
        e2e_latencies = [row[1] for row in detections if row[1] is not None]
        inference_times = [row[2] for row in detections]
        detection_counts = [row[3] for row in detections]
        decode_latencies = [row[1] for row in decode]

        summary = {
            "mode": self.mode,
            "label": (
                "JPEG over ROS + pure YOLO" if self.mode == "baseline"
                else "motion gate + H.264 RTP/UDP + ByteTrack + feedback"
            ),
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
                # 基线模式：采集 -> 边缘端接收；改进模式：采集 -> 编码并发送
                "transport_stage": stats_block(transport_latencies),
                # 仅改进模式：采集 -> 边缘端解码
                "capture_to_decoded": stats_block(decode_latencies),
                # 两种模式：采集 -> 检测消息发布
                "end_to_end_detection": stats_block(e2e_latencies),
            },
            "inference_ms": stats_block(inference_times),
            "detections_per_frame": stats_block(detection_counts),
            "clock_sync_note": (
                "Cross-device latencies assume endpoint and edge clocks are "
                "synchronized via chrony/NTP."
            ),
        }

        if self.mode == "improved":
            summary["gate"] = gate
            summary["feedback"] = {
                "control_messages": len(controls),
                "profile_switches": self.count_profile_switches(controls),
                "last_events": controls[-5:],
            }
            summary["tracking"] = {
                "frames_with_tracks": sum(1 for c in tracks if c > 0),
                "avg_tracks": (
                    round(float(np.mean(tracks)), 2) if tracks else 0.0
                ),
            }
            if settings:
                fps_values = [s[0] for s in settings]
                bitrate_values = [s[1] for s in settings]
                summary["adaptive_settings"] = {
                    "fps_min": min(fps_values), "fps_max": max(fps_values),
                    "bitrate_kbps_min": min(bitrate_values),
                    "bitrate_kbps_max": max(bitrate_values),
                }
        return summary, transport, detections

    @staticmethod
    def count_profile_switches(controls):
        switches = 0
        previous = None
        for event in controls:
            key = (event["target_fps"], event["target_bitrate_kbps"])
            if previous is not None and key != previous:
                switches += 1
            previous = key
        return switches

    def write_results(self):
        summary, transport, detections = self.build_summary()
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        run_dir = os.path.join(self.results_dir, "%s_%s" % (self.mode, timestamp))
        try:
            os.makedirs(run_dir)
        except OSError:
            pass

        with open(os.path.join(run_dir, "summary.json"), "w") as handle:
            json.dump(summary, handle, indent=2, ensure_ascii=False)

        with open(os.path.join(run_dir, "frames.csv"), "w") as handle:
            writer = csv.writer(handle)
            writer.writerow(["type", "wall_time", "value_1", "value_2", "value_3"])
            for row in transport:
                writer.writerow(
                    ["transport", "%.3f" % row[0], row[1],
                     "" if row[2] is None else "%.2f" % row[2], ""]
                )
            for row in detections:
                writer.writerow(
                    ["detection", "%.3f" % row[0],
                     "" if row[1] is None else "%.2f" % row[1],
                     "%.2f" % row[2], row[3]]
                )

        print("[metrics_recorder] results written to %s" % run_dir)

        if self.generate_comparison:
            self.write_comparison(summary, timestamp)

    def find_latest_other_run(self):
        other_mode = "improved" if self.mode == "baseline" else "baseline"
        pattern = os.path.join(self.results_dir, "%s_*" % other_mode, "summary.json")
        candidates = sorted(glob.glob(pattern))
        if not candidates:
            return None
        with open(candidates[-1]) as handle:
            return json.load(handle)

    def write_comparison(self, current, timestamp):
        other = self.find_latest_other_run()
        if other is None:
            print(
                "[metrics_recorder] no %s run found yet - comparison skipped"
                % ("improved" if self.mode == "baseline" else "baseline")
            )
            return

        baseline = current if current["mode"] == "baseline" else other
        improved = current if current["mode"] == "improved" else other

        def metric(source, *path):
            node = source
            for key in path:
                if not isinstance(node, dict) or key not in node:
                    return None
                node = node[key]
            return node

        def row(name, path, unit, lower_is_better=True):
            b = metric(baseline, *path)
            i = metric(improved, *path)
            if b in (None, 0) or i is None:
                delta = "-"
            else:
                change = (i - b) / abs(b) * 100.0
                better = (change < 0) == lower_is_better
                delta = "%+.1f%% %s" % (change, "✓" if better else "✗")
            fmt = lambda v: "-" if v is None else ("%.2f" % v)
            return "| %s | %s | %s | %s |" % (
                "%s (%s)" % (name, unit), fmt(b), fmt(i), delta
            )

        lines = [
            "# 新旧方案性能对比报告",
            "",
            "生成时间: %s" % time.strftime("%Y-%m-%d %H:%M:%S"),
            "",
            "- **baseline**: %s (运行于 %s, 时长 %ss)" % (
                baseline["label"], baseline["started_at"], baseline["duration_s"]
            ),
            "- **improved**: %s (运行于 %s, 时长 %ss)" % (
                improved["label"], improved["started_at"], improved["duration_s"]
            ),
            "",
            "| 指标 | baseline | improved | 变化 |",
            "| --- | --- | --- | --- |",
            row("平均带宽", ("transport", "avg_kbps"), "kbps"),
            row("单帧平均传输量", ("transport", "avg_bytes_per_frame"), "bytes"),
            row("传输帧率", ("transport", "avg_fps"), "fps", lower_is_better=False),
            row("端到端检测延迟-均值",
                ("latency_ms", "end_to_end_detection", "mean"), "ms"),
            row("端到端检测延迟-中位数",
                ("latency_ms", "end_to_end_detection", "median"), "ms"),
            row("端到端检测延迟-P95",
                ("latency_ms", "end_to_end_detection", "p95"), "ms"),
            row("推理耗时-均值", ("inference_ms", "mean"), "ms"),
            row("推理耗时-P95", ("inference_ms", "p95"), "ms"),
            row("平均检测数/帧", ("detections_per_frame", "mean"), "个",
                lower_is_better=False),
            "",
        ]

        gate = improved.get("gate")
        if gate:
            lines.append(
                "**内容感知过滤**: 端侧共采集 %d 帧, 转发 %d 帧 (通过率 %.1f%%), "
                "静止画面被直接丢弃以节省带宽。"
                % (gate["frames_in"], gate["frames_passed"],
                   gate["pass_ratio"] * 100.0)
            )
        feedback = improved.get("feedback")
        if feedback:
            lines.append(
                "**反馈闭环**: 边缘侧共下发 %d 条码率/帧率控制指令, 档位切换 %d 次。"
                % (feedback["control_messages"], feedback["profile_switches"])
            )
        settings = improved.get("adaptive_settings")
        if settings:
            lines.append(
                "**自适应范围**: 帧率 %.1f-%.1f fps, 码率 %d-%d kbps。"
                % (settings["fps_min"], settings["fps_max"],
                   settings["bitrate_kbps_min"], settings["bitrate_kbps_max"])
            )
        lines.append("")
        lines.append(
            "> 注: 跨设备延迟依赖两台设备时钟同步 (chrony/NTP); "
            "对比公平性要求两次运行使用相同权重、分辨率与相似场景。"
        )
        lines.append("")

        report_path = os.path.join(
            self.results_dir, "comparison_%s.md" % timestamp
        )
        with open(report_path, "w") as handle:
            handle.write("\n".join(lines))
        with open(
            os.path.join(self.results_dir, "comparison_%s.json" % timestamp), "w"
        ) as handle:
            json.dump(
                {"baseline": baseline, "improved": improved},
                handle, indent=2, ensure_ascii=False,
            )
        print("[metrics_recorder] comparison report written to %s" % report_path)


def main():
    rospy.init_node("metrics_recorder")
    MetricsRecorder()
    rospy.spin()


if __name__ == "__main__":
    main()
