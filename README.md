# 双设备视频流边缘计算系统（ROS1 + YOLO）

本项目按照"端侧设备 A 采集与预处理、边缘设备 B 执行 AI 推理"的方式构建,
并提供 **baseline / improved 两套可切换的完整链路**,用于量化对比。

**baseline（原始方案）**: 缩放降帧 + JPEG 逐帧压缩,经 ROS 话题传输,边缘侧纯 YOLO 检测。

**improved（改进方案）**: 内容感知过滤 + H.264 硬件编码 RTP/UDP 带外传输,
边缘侧 ByteTrack 多目标跟踪 + 轨迹预测,并将帧率/码率控制指令反馈回端侧,构成闭环。

~~~text
                         improved 数据流

端侧设备 A (Jetson Nano)                    边缘设备 B (Ubuntu VM)
USB 摄像头
  │ /usb_cam/image_raw
  ▼
motion_gate  内容感知过滤(MOG2 运动检测,静止帧丢弃)
  │ /uav/image_gated            ├─► /uav/gate_stats (ROS)
  ▼
h264_sender  H.264 硬编码(nvv4l2h264enc,回退 x264enc)
  │                             ├─► /uav/stream_meta (ROS, 逐帧时间戳/字节数)
  ├────── RTP/UDP :5600 ──────────► h264_receiver ─► /uav/image_decoded
  │                                    ▼
  │                                 yolo_tracker  YOLO + ByteTrack
  │                                    ├─► /yolo/detections /yolo/tracks
  │                                    ├─► /yolo/annotated_image (轨迹+预测点)
  ◄──── /uav/stream_control (ROS) ─────┘  反馈闭环:自适应帧率/码率
                                       ▼
                                    metrics_recorder ─► results/ 对比报告
~~~

视频本体不占用 ROS 带宽(带外 RTP/UDP);逐帧元数据、门控统计与反馈控制仍走
ROS 话题,所有消息都带时间戳,可端到端统计延迟。边缘侧解码后重新发布标准
`sensor_msgs/Image`,下游任何 ROS 节点(rqt、rosbag、其他算法)均不受影响。

## 项目结构

~~~text
endpoint_device/                 端侧设备 A 的 catkin 工作空间
└─ src/
   ├─ robot_vision/              采集、预处理、运动门控、H.264 发送
   │  └─ scripts/
   │     ├─ image_preprocessor.py   baseline: 缩放/降帧/JPEG
   │     ├─ motion_gate.py          improved: 内容感知过滤
   │     └─ h264_sender.py          improved: 硬编码 + RTP/UDP + 自适应
   ├─ stream_msgs/               共享消息(StreamMeta/StreamControl/GateStats)
   └─ usb_cam/                   第三方 V4L USB 摄像头驱动

edge_device/                     边缘设备 B 的 catkin 工作空间
└─ src/
   ├─ edge_yolo_ros/
   │  └─ scripts/
   │     ├─ yolo_detector.py        baseline: 纯 YOLO 检测
   │     ├─ h264_receiver.py        improved: RTP 接收解码,恢复采集时间戳
   │     ├─ yolo_tracker.py         improved: ByteTrack + 轨迹预测 + 反馈
   │     └─ metrics_recorder.py     两种模式通用:指标统计与对比报告
   └─ stream_msgs/               与端侧完全相同的副本(MD5 必须一致,勿单独改)
~~~

部署到设备后,文档后续分别使用 **~/endpoint_ws** 和 **~/edge_ws** 作为工作空间。

## 1. 网络配置与时钟同步

两台设备必须位于可互通的局域网。虚拟机作为边缘设备时,网卡应使用桥接模式。
假设 ROS Master 运行在端侧设备:

~~~text
端侧设备 A IP    192.168.0.100
边缘设备 B IP    192.168.0.118
ROS Master       http://192.168.0.100:11311
RTP 视频流       udp://192.168.0.118:5600 (improved 模式)
~~~

端侧设备:

~~~bash
export ROS_MASTER_URI=http://192.168.0.100:11311
export ROS_IP=192.168.0.100
~~~

边缘设备:

~~~bash
export ROS_MASTER_URI=http://192.168.0.100:11311
export ROS_IP=192.168.0.118
~~~

**跨设备延迟统计依赖两台设备时钟同步**,否则"采集→检测"延迟无意义。
推荐用 chrony,让边缘侧对齐端侧(或同一 NTP 源):

~~~bash
sudo apt install chrony
# 边缘侧 /etc/chrony/chrony.conf 中加入: server 192.168.0.100 iburst prefer
chronyc tracking   # 确认偏差在毫秒级
~~~

## 2. 部署端侧设备 A

端侧需要 ROS1 Noetic、catkin、cv_bridge、V4L2 USB 摄像头;improved 模式还需要
GStreamer Python 绑定(Jetson 镜像一般自带):

~~~bash
sudo apt install python3-gi gstreamer1.0-tools \
  gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly
gst-inspect-1.0 nvv4l2h264enc   # Jetson 上确认硬件编码器可用
~~~

将 **endpoint_device** 复制为 **~/endpoint_ws**,然后:

~~~bash
cd ~/endpoint_ws
catkin_init_workspace src
chmod +x src/robot_vision/scripts/image_preprocessor.py \
         src/robot_vision/scripts/motion_gate.py \
         src/robot_vision/scripts/h264_sender.py
catkin_make
source devel/setup.bash
~~~

启动(二选一):

~~~bash
# baseline: JPEG over ROS
roslaunch robot_vision endpoint.launch mode:=baseline

# improved: 运动门控 + H.264 RTP/UDP(edge_host 填边缘侧 IP)
roslaunch robot_vision endpoint.launch mode:=improved edge_host:=192.168.0.118
~~~

baseline 默认输出 416×312、10 FPS、JPEG 质量 70,可启动时调整:

~~~bash
roslaunch robot_vision endpoint.launch mode:=baseline \
  output_width:=640 output_height:=480 output_fps:=10 jpeg_quality:=70
~~~

improved 模式常用参数: `motion_threshold`(运动像素比阈值,默认 0.005)、
`keepalive_interval`(静止时保底发送间隔,默认 2s)、`initial_fps`、
`initial_bitrate_kbps`;非 Jetson 设备调试时加 `force_software_encoder:=true`。

验证(baseline 看 ROS 话题,improved 看元数据):

~~~bash
rostopic hz /uav/image_preprocessed/compressed   # baseline
rostopic hz /uav/stream_meta                     # improved
~~~

## 3. 部署边缘设备 B

推荐 Ubuntu 20.04、ROS Noetic、Python 3。improved 模式需要 GStreamer 解码组件:

~~~bash
sudo apt install python3-gi gstreamer1.0-tools \
  gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-libav
~~~

将 **edge_device** 复制为 **~/edge_ws**,创建 Python 环境(CPU 为例):

~~~bash
source /opt/ros/noetic/setup.bash
python3 -m venv --system-site-packages ~/venvs/edge_yolo
source ~/venvs/edge_yolo/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
python3 -m pip install -r ~/edge_ws/src/edge_yolo_ros/requirements.txt
python3 -c "import rospy, torch, ultralytics; print('environment OK')"
~~~

编译:

~~~bash
chmod +x ~/edge_ws/src/edge_yolo_ros/scripts/*.py
cd ~/edge_ws
catkin_init_workspace src
catkin_make -DPYTHON_EXECUTABLE=$VIRTUAL_ENV/bin/python3
source devel/setup.bash
~~~

启动(与端侧模式保持一致):

~~~bash
# baseline
roslaunch edge_yolo_ros edge.launch mode:=baseline device:=cpu

# improved
roslaunch edge_yolo_ros edge.launch mode:=improved device:=cpu
~~~

CUDA 推理时 `device:=0`;自定义权重 `weights:=/path/to/best.pt`。
improved 模式可调 `prediction_horizon_s`(轨迹预测时域)、
`active_fps`/`active_bitrate_kbps`/`idle_fps`/`idle_bitrate_kbps`/`idle_timeout`
(反馈闭环的两档配置)。

仓库当前附带 yolov8n.pt 和 yolo26n.pt,默认使用 yolov8n.pt。模型权重及
Ultralytics 软件受其各自许可约束,上传或商用前请确认适用条款。

## 4. 性能对比流程

1. 两侧以 `mode:=baseline` 运行一段时间(建议 ≥60s,场景中有目标活动),
   Ctrl-C 结束边缘侧 —— `metrics_recorder` 自动写入
   `~/edge_yolo_ros/results/baseline_<时间戳>/{summary.json, frames.csv}`。
2. 两侧改为 `mode:=improved` 在相似场景重复一次。
3. 每次运行结束时,若已存在另一模式的结果,自动生成
   `~/edge_yolo_ros/results/comparison_<时间戳>.md`(及 .json),
   对比平均带宽、单帧传输量、传输帧率、端到端延迟(均值/中位数/P95)、
   推理耗时等,并附内容感知过滤通过率、反馈闭环切换次数与自适应范围。

公平对比注意:两次运行使用相同权重与 `image_size`、相似的场景与时长;
时钟未同步时延迟栏仅供参考,带宽/帧率/推理耗时不受影响。

## 5. 验证与常见问题

在边缘设备的新终端中加载 ROS 与工作空间环境后执行:

~~~bash
rostopic list
rostopic echo /yolo/detections
rqt_image_view /yolo/annotated_image  # 检测框;improved 另有轨迹+预测点
# improved 专属:
rostopic hz /uav/image_decoded        # 解码后的图像流
rostopic echo /uav/gate_stats         # 门控通过率
rostopic echo /uav/stream_control     # 反馈闭环指令
~~~

1. 看不到端侧话题:检查 ROS_MASTER_URI、两端 ROS_IP、防火墙和双向 ping。
2. 有话题但没有图像:确认 USB 摄像头节点正常,并检查网络带宽。
3. improved 模式无图像:确认 UDP 5600 未被防火墙拦截
   (`sudo ufw allow 5600/udp`),并在边缘侧用
   `gst-launch-1.0 udpsrc port=5600 caps="application/x-rtp,media=video,encoding-name=H264,payload=96" ! rtpjitterbuffer ! rtph264depay ! avdec_h264 ! autovideosink` 单独验证链路。
4. 编码器报错:非 Jetson 端侧加 `force_software_encoder:=true`。
5. YOLO/跟踪导入失败:确认编译和运行使用同一个 Python 虚拟环境,
   且已安装 `lapx`(ByteTrack 依赖)。
6. 修改 stream_msgs 后:两个工作空间内的副本必须同步修改并分别重新编译,
   否则消息 MD5 不一致会导致话题无法通信。

## 6. 配置入口

- 端侧启动参数: **endpoint_device/src/robot_vision/launch/endpoint.launch**
- 边缘启动参数: **edge_device/src/edge_yolo_ros/launch/edge.launch**
- baseline 推理参数: **edge_device/src/edge_yolo_ros/config/detector.yaml**
- 共享消息定义: **endpoint_device/src/stream_msgs/msg/**
- YOLO 包详细说明: **edge_device/src/edge_yolo_ros/README.md**

## 许可证

本仓库包含不同来源的组件,不使用单一许可证覆盖全部内容:

- edge_yolo_ros: GNU AGPL-3.0-only。
- robot_vision: MIT。
- stream_msgs: MIT。
- usb_cam: BSD 3-Clause,必须保留其原始 LICENSE 和源码版权声明。
- Ultralytics 及模型权重:遵循其上游许可或商业授权。

完整归属与路径见 **THIRD_PARTY_NOTICES.md**。
