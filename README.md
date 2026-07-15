# 双设备视频流边缘计算系统（ROS1 + YOLO）

本项目由端侧设备 A 采集视频，边缘设备 B 执行 AI 推理，提供两套可切换链路：

- `baseline`：缩放、限帧和 JPEG 压缩后通过 ROS 话题传输，边缘侧执行 YOLO 检测。
- `improved`：运动门控后使用 H.264 RTP/UDP 传输，边缘侧执行 YOLO + ByteTrack，并通过反馈控制动态调整帧率和码率。

`master` 分支保留 baseline；`dev` 分支包含 baseline 和 improved 的完整实现。

## 数据流

~~~text
baseline

USB 摄像头
  -> image_compressor
  -> /uav/image_preprocessed/compressed (ROS)
  -> yolo_detector
  -> /yolo/detections + /yolo/annotated_image

improved

USB 摄像头
  -> motion_gate
  -> h264_sender
  -> H.264 RTP/UDP :5600
  -> h264_receiver
  -> /uav/image_decoded
  -> yolo_tracker
  -> /yolo/detections + /yolo/tracks + /yolo/annotated_image

/uav/stream_control: 边缘侧向端侧反馈目标帧率和码率
/uav/stream_meta、/uav/gate_stats: 传输元数据和门控统计
~~~

## 项目结构

~~~text
endpoint_device/                         端侧 catkin 工作空间
└─ src/
   ├─ robot_vision/
   │  ├─ launch/endpoint.launch         两种链路的统一启动文件
   │  └─ scripts/
   │     ├─ image_compressor.py         baseline 图像预处理
   │     ├─ motion_gate.py              improved 运动门控
   │     └─ h264_sender.py              improved H.264 发送与控制
   ├─ stream_msgs/                      improved 共享消息
   └─ usb_cam/                          V4L USB 摄像头驱动

edge_device/                             边缘侧 catkin 工作空间
└─ src/
   ├─ edge_yolo_ros/
   │  ├─ launch/edge.launch             两种链路的统一启动文件
   │  └─ scripts/
   │     ├─ yolo_detector.py            baseline YOLO 检测
   │     ├─ h264_receiver.py            improved H.264 接收与解码
   │     ├─ yolo_tracker.py             improved 跟踪、预测与反馈
   │     └─ metrics_recorder.py         指标记录与对比报告
   └─ stream_msgs/                      与端侧一致的消息定义
~~~

## 1. 网络与时钟

两台设备需要位于可互通的局域网。以下配置假设 ROS Master 运行在端侧：

~~~text
端侧设备 A IP    192.168.0.100
边缘设备 B IP    192.168.0.118
ROS Master       http://192.168.0.100:11311
RTP 视频流       udp://192.168.0.118:5600
~~~

端侧设备：

~~~bash
export ROS_MASTER_URI=http://192.168.0.100:11311
export ROS_IP=192.168.0.100
~~~

边缘设备：

~~~bash
export ROS_MASTER_URI=http://192.168.0.100:11311
export ROS_IP=192.168.0.118
~~~

跨设备延迟统计要求两台设备时钟同步。可使用 chrony 对齐到同一 NTP 源，并通过
`chronyc tracking` 检查偏差。

## 2. 端侧部署

端侧需要 ROS1 Noetic、catkin、cv_bridge 和 V4L2。运行 improved 还需要 GStreamer：

~~~bash
sudo apt install python3-gi gstreamer1.0-tools \
  gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly
~~~

将 `endpoint_device` 作为 `~/endpoint_ws`，然后编译：

~~~bash
cd ~/endpoint_ws
catkin_init_workspace src
catkin_make
source devel/setup.bash
~~~

启动 baseline：

~~~bash
roslaunch robot_vision endpoint.launch mode:=baseline
~~~

默认输出为 416×312、10 FPS、JPEG 质量 70，可按需调整：

~~~bash
roslaunch robot_vision endpoint.launch mode:=baseline \
  output_width:=640 output_height:=480 output_fps:=10 jpeg_quality:=70
~~~

启动 improved，其中 `edge_host` 为边缘设备 IP：

~~~bash
roslaunch robot_vision endpoint.launch mode:=improved edge_host:=192.168.0.118
~~~

非 Jetson 设备可增加 `force_software_encoder:=true` 使用 x264；Jetson 可通过
`gst-inspect-1.0 nvv4l2h264enc` 检查硬件编码器。

## 3. 边缘侧部署

推荐 Ubuntu 20.04、ROS Noetic 和 Python 3。先安装适合 CPU 或 CUDA 环境的
PyTorch，再安装项目依赖：

~~~bash
source /opt/ros/noetic/setup.bash
python3 -m venv --system-site-packages ~/venvs/edge_yolo
source ~/venvs/edge_yolo/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
python3 -m pip install -r ~/edge_ws/src/edge_yolo_ros/requirements.txt
~~~

运行 improved 时还需要 GStreamer 解码组件：

~~~bash
sudo apt install python3-gi gstreamer1.0-tools \
  gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-libav
~~~

将 `edge_device` 作为 `~/edge_ws`，然后编译：

~~~bash
cd ~/edge_ws
catkin_init_workspace src
catkin_make -DPYTHON_EXECUTABLE=$VIRTUAL_ENV/bin/python3
source devel/setup.bash
~~~

启动时，两侧的 `mode` 必须保持一致：

~~~bash
# baseline，CPU 推理
roslaunch edge_yolo_ros edge.launch mode:=baseline device:=cpu

# improved，CPU 推理
roslaunch edge_yolo_ros edge.launch mode:=improved device:=cpu
~~~

CUDA 推理使用 `device:=0`。自定义模型可通过
`weights:=/path/to/best.pt` 指定。

## 4. 参数与验证

常用 improved 参数：

- 端侧：`motion_threshold`、`keepalive_interval`、`initial_fps`、`initial_bitrate_kbps`。
- 边缘侧：`prediction_horizon_s`、`active_fps`、`active_bitrate_kbps`、`idle_fps`、`idle_bitrate_kbps`、`idle_timeout`。

常用验证命令：

~~~bash
rostopic echo /yolo/detections
rqt_image_view /yolo/annotated_image

# baseline
rostopic hz /uav/image_preprocessed/compressed

# improved
rostopic hz /uav/image_decoded
rostopic echo /uav/gate_stats
rostopic echo /uav/stream_control
~~~

`metrics_recorder` 会在退出时将运行数据写入
`~/edge_yolo_ros/results/<mode>_<时间戳>/`。baseline 和 improved 都完成一次后，会生成
Markdown 与 JSON 对比报告。

## 5. 常见问题

1. 看不到跨设备话题：检查 `ROS_MASTER_URI`、`ROS_IP`、防火墙和双向 ping。
2. baseline 无图像：检查摄像头节点及 `/uav/image_preprocessed/compressed`。
3. improved 无图像：检查 UDP 5600、防火墙及两侧 GStreamer 插件。
4. improved 编码失败：Jetson 检查 `nvv4l2h264enc`；其他设备启用软件编码。
5. YOLO 或 ByteTrack 导入失败：确认编译与运行使用同一 Python 环境，并已安装
   `requirements.txt` 中的依赖。
6. 修改 `stream_msgs` 后：端侧与边缘侧的消息定义必须保持一致，并分别重新编译。

## 许可证

- `edge_yolo_ros`：GNU AGPL-3.0-only。
- `robot_vision`：MIT。
- `stream_msgs`：MIT。
- `usb_cam`：BSD 3-Clause。
- Ultralytics 与模型权重遵循各自适用的许可条款。

第三方组件与许可路径见 `THIRD_PARTY_NOTICES.md`。
