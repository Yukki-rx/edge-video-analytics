#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""采用 H.264 硬件编码的 RTP/UDP 视频发送器（带外传输）。

接收经过门控的相机帧，在可用时使用 Jetson 硬件编码器
（nvv4l2h264enc）进行编码；在其他机器上则回退到 x264enc，随后通过
UDP 将 RTP/H.264 视频流发送至边缘设备。视频本身不经过 ROS 传输；
每帧元数据（采集时间戳、编码后大小、当前帧率/码率设置）会发布到
ROS 话题，使边缘端能够还原延迟和带宽统计信息。

监听 stream_msgs/StreamControl，以接收边缘设备给出的自适应帧率/码率
反馈（闭环）。
"""

import collections
import threading

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from stream_msgs.msg import StreamControl, StreamMeta

import gi
gi.require_version("Gst", "1.0")
from gi.repository import GLib, Gst


class H264Sender(object):
    def __init__(self):
        Gst.init(None)

        input_topic = rospy.get_param("~input_topic", "/uav/image_gated")
        meta_topic = rospy.get_param("~meta_topic", "/uav/stream_meta")
        control_topic = rospy.get_param("~control_topic", "/uav/stream_control")

        self.edge_host = rospy.get_param("~edge_host", "192.168.0.118")
        self.port = int(rospy.get_param("~port", 5600))
        self.width = int(rospy.get_param("~output_width", 640))
        self.height = int(rospy.get_param("~output_height", 480))
        self.target_fps = float(rospy.get_param("~initial_fps", 10.0))
        self.bitrate_kbps = int(rospy.get_param("~initial_bitrate_kbps", 1200))
        self.iframe_interval = int(rospy.get_param("~iframe_interval", 15))
        self.force_software = bool(rospy.get_param("~force_software_encoder", False))

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self.last_push_time = rospy.Time(0)
        self.seq = 0
        # 等待与编码器输出匹配的 (seq, capture_stamp)。
        self.pending = collections.deque(maxlen=64)

        self.build_pipeline()

        self.meta_publisher = rospy.Publisher(meta_topic, StreamMeta, queue_size=10)
        self.control_subscriber = rospy.Subscriber(
            control_topic, StreamControl, self.control_callback, queue_size=1
        )
        self.image_subscriber = rospy.Subscriber(
            input_topic, Image, self.image_callback, queue_size=1, buff_size=2 ** 24
        )

        self.loop = GLib.MainLoop()
        self.loop_thread = threading.Thread(target=self.loop.run)
        self.loop_thread.daemon = True
        self.loop_thread.start()
        rospy.on_shutdown(self.shutdown)

        rospy.loginfo(
            "H264 sender: %s -> udp://%s:%d (%dx%d, %.1f fps, %d kbps, encoder=%s)",
            input_topic, self.edge_host, self.port, self.width, self.height,
            self.target_fps, self.bitrate_kbps, self.encoder_name,
        )

    # ------------------------------------------------------------------ #
    # GStreamer 流水线
    # ------------------------------------------------------------------ #
    def build_pipeline(self):
        use_nv = (
            not self.force_software
            and Gst.ElementFactory.find("nvv4l2h264enc") is not None
        )
        caps = (
            "video/x-raw,format=BGR,width=%d,height=%d,framerate=0/1"
            % (self.width, self.height)
        )
        if use_nv:
            self.encoder_name = "nvv4l2h264enc"
            encode = (
                "videoconvert ! video/x-raw,format=I420 ! nvvidconv ! "
                "video/x-raw(memory:NVMM),format=I420 ! "
                "nvv4l2h264enc name=enc bitrate=%d insert-sps-pps=true "
                "iframeinterval=%d idrinterval=%d maxperf-enable=1"
                % (self.bitrate_kbps * 1000, self.iframe_interval, self.iframe_interval)
            )
        else:
            self.encoder_name = "x264enc"
            encode = (
                "videoconvert ! video/x-raw,format=I420 ! "
                "x264enc name=enc tune=zerolatency speed-preset=ultrafast "
                "bitrate=%d key-int-max=%d"
                % (self.bitrate_kbps, self.iframe_interval)
            )

        description = (
            "appsrc name=src is-live=true do-timestamp=true format=time caps=\"%s\" ! "
            "%s ! h264parse config-interval=1 ! "
            "rtph264pay pt=96 mtu=1200 ! "
            "udpsink host=%s port=%d sync=false"
            % (caps, encode, self.edge_host, self.port)
        )
        rospy.loginfo("GStreamer pipeline: %s", description)

        self.pipeline = Gst.parse_launch(description)
        self.appsrc = self.pipeline.get_by_name("src")
        self.encoder = self.pipeline.get_by_name("enc")

        # 探测编码器输出：每个编码帧对应一个缓冲区。按照先进先出顺序
        # 与待处理的采集时间戳配对，以发布逐帧元数据。
        encoder_src_pad = self.encoder.get_static_pad("src")
        encoder_src_pad.add_probe(Gst.PadProbeType.BUFFER, self.on_encoded_buffer)

        self.pipeline.set_state(Gst.State.PLAYING)

    def on_encoded_buffer(self, _pad, info):
        buffer = info.get_buffer()
        if buffer is None:
            return Gst.PadProbeReturn.OK
        try:
            seq, capture_stamp = self.pending.popleft()
        except IndexError:
            return Gst.PadProbeReturn.OK

        meta = StreamMeta()
        meta.header.stamp = capture_stamp
        meta.header.frame_id = "uav_camera"
        meta.seq = seq
        meta.sent_stamp = rospy.Time.now()
        meta.bytes = buffer.get_size()
        with self.lock:
            meta.fps_setting = self.target_fps
            meta.bitrate_kbps = self.bitrate_kbps
        self.meta_publisher.publish(meta)
        return Gst.PadProbeReturn.OK

    # ------------------------------------------------------------------ #
    # ROS 回调
    # ------------------------------------------------------------------ #
    def image_callback(self, message):
        now = rospy.Time.now()
        with self.lock:
            interval = 1.0 / self.target_fps if self.target_fps > 0 else 1.0
        if (now - self.last_push_time).to_sec() < interval:
            return

        try:
            frame = self.bridge.imgmsg_to_cv2(message, "bgr8")
        except CvBridgeError as error:
            rospy.logerr_throttle(5.0, "cv_bridge conversion failed: %s", error)
            return

        if frame.shape[1] != self.width or frame.shape[0] != self.height:
            frame = cv2.resize(
                frame, (self.width, self.height), interpolation=cv2.INTER_AREA
            )

        data = frame.tobytes()
        buffer = Gst.Buffer.new_wrapped(data)

        capture_stamp = message.header.stamp
        if capture_stamp == rospy.Time(0):
            capture_stamp = now
        self.pending.append((self.seq, capture_stamp))
        self.seq += 1

        result = self.appsrc.emit("push-buffer", buffer)
        if result != Gst.FlowReturn.OK:
            rospy.logwarn_throttle(5.0, "appsrc push-buffer returned %s", result)
            try:
                self.pending.pop()
            except IndexError:
                pass
            return
        self.last_push_time = now

    def control_callback(self, message):
        with self.lock:
            changed = []
            if message.target_fps > 0 and message.target_fps != self.target_fps:
                self.target_fps = float(message.target_fps)
                changed.append("fps=%.1f" % self.target_fps)
            if (
                message.target_bitrate_kbps > 0
                and message.target_bitrate_kbps != self.bitrate_kbps
            ):
                self.bitrate_kbps = int(message.target_bitrate_kbps)
                changed.append("bitrate=%dkbps" % self.bitrate_kbps)
                self.apply_bitrate()
        if changed:
            rospy.loginfo(
                "Stream control applied: %s (%s)", ", ".join(changed), message.reason
            )

    def apply_bitrate(self):
        try:
            if self.encoder_name == "nvv4l2h264enc":
                self.encoder.set_property("bitrate", self.bitrate_kbps * 1000)
            else:
                self.encoder.set_property("bitrate", self.bitrate_kbps)
        except Exception as error:
            rospy.logwarn("Failed to change encoder bitrate at runtime: %s", error)

    def shutdown(self):
        try:
            self.appsrc.emit("end-of-stream")
            self.pipeline.set_state(Gst.State.NULL)
            self.loop.quit()
        except Exception:
            pass


def main():
    rospy.init_node("h264_sender")
    H264Sender()
    rospy.spin()


if __name__ == "__main__":
    main()
