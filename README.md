# Edge Video Analytics

基于 ROS1 与 Ultralytics YOLO 的双端视频分析示例。端侧（`endpoint`）负责从 USB 摄像头读取图像，并完成降帧、缩放与 JPEG 压缩；边缘侧（`edge`）通过局域网接收压缩图像，执行 YOLO 检测并发布结构化结果和标注图像。

```text
端侧 / endpoint                                边缘侧 / edge
USB 摄像头 -> /usb_cam/image_raw               YOLO 推理
           -> 缩放、限帧、JPEG 压缩    ROS1    -> /yolo/detections
           -> /uav/image_preprocessed/compressed ---> /yolo/annotated_image
```

## 仓库结构

本仓库包含两个可独立编译、部署在不同设备上的 catkin 工作空间：

```text
.
├── endpoint/                         # 摄像头采集与图像预处理工作空间
│   └── src/
│       ├── robot_vision/
│       │   ├── launch/
│       │   │   └── edge_camera.launch
│       │   └── scripts/
│       │       └── image_preprocessor.py
│       └── usb_cam/                  # ROS USB 摄像头驱动
├── edge/                             # YOLO 推理工作空间
│   └── src/
│       └── edge_yolo_ros/
│           ├── config/detector.yaml
│           ├── launch/yolo_detector.launch
│           ├── msg/                  # Detection、DetectionArray
│           ├── scripts/yolo_detector.py
│           └── weights/yolov8n.pt    # 默认 COCO 模型
├── README.md
└── THIRD_PARTY_NOTICES.md
```

## 运行环境

- 两台处于同一局域网、能够双向访问的 Linux 设备
- ROS1 与 catkin（以下命令以 Ubuntu 20.04 + ROS Noetic 为例）
- 端侧：USB 摄像头、OpenCV、`cv_bridge`、`image_transport`
- 边缘侧：Python 3、OpenCV、NumPy、PyTorch、Ultralytics 8.x

虚拟机作为边缘侧时，网卡应使用桥接模式。PyTorch 必须根据实际硬件安装 CPU 或 CUDA 版本。

## 1. 配置 ROS 多机通信

假设端侧 IP 为 `192.168.1.10`，边缘侧 IP 为 `192.168.1.20`，并在端侧运行 ROS Master。

端侧：

```bash
export ROS_MASTER_URI=http://192.168.1.10:11311
export ROS_IP=192.168.1.10
```

边缘侧：

```bash
export ROS_MASTER_URI=http://192.168.1.10:11311
export ROS_IP=192.168.1.20
```

先确认两端可以互相 `ping`，再把对应变量写入各自的 `~/.bashrc`。如果使用主机名代替 IP，还需保证两端都能正确解析该主机名。

## 2. 构建并启动端侧

将仓库中的 `endpoint` 目录复制到采集设备，然后执行：

```bash
source /opt/ros/noetic/setup.bash
chmod +x ~/edge-video-analytics/endpoint/src/robot_vision/scripts/image_preprocessor.py
cd ~/edge-video-analytics/endpoint
catkin_init_workspace src
catkin_make
source devel/setup.bash
roslaunch robot_vision edge_camera.launch
```

默认使用 `/dev/video0`，采集分辨率为 640×480；预处理后以 416×312、10 FPS、JPEG 质量 70 发布。可以在启动时覆盖参数：

```bash
roslaunch robot_vision edge_camera.launch \
  video_device:=/dev/video0 \
  camera_width:=640 camera_height:=480 \
  output_width:=640 output_height:=480 \
  output_fps:=10 jpeg_quality:=70
```

验证端侧输出：

```bash
rostopic type /uav/image_preprocessed/compressed
rostopic hz /uav/image_preprocessed/compressed
```

话题类型应为 `sensor_msgs/CompressedImage`。

## 3. 构建并启动边缘侧

将仓库中的 `edge` 目录复制到推理设备。建议创建可同时访问 ROS Python 包和 YOLO 依赖的虚拟环境：

```bash
source /opt/ros/noetic/setup.bash
python3 -m venv --system-site-packages ~/venvs/edge_yolo
source ~/venvs/edge_yolo/bin/activate
python3 -m pip install --upgrade pip

# 以下为 CPU 版本；CUDA 环境请改用与本机驱动匹配的 PyTorch 安装方式
python3 -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
python3 -m pip install -r ~/edge-video-analytics/edge/src/edge_yolo_ros/requirements.txt
python3 -c "import rospy, cv2, torch, ultralytics; print('environment OK')"
```

使用该 Python 环境编译工作空间：

```bash
chmod +x ~/edge-video-analytics/edge/src/edge_yolo_ros/scripts/yolo_detector.py
cd ~/edge-video-analytics/edge
catkin_init_workspace src
catkin_make -DPYTHON_EXECUTABLE="$VIRTUAL_ENV/bin/python3"
source devel/setup.bash
```

启动 CPU 推理：

```bash
source /opt/ros/noetic/setup.bash
source ~/venvs/edge_yolo/bin/activate
source ~/edge-video-analytics/edge/devel/setup.bash
roslaunch edge_yolo_ros yolo_detector.launch device:=cpu
```

默认加载仓库内的 `weights/yolov8n.pt`。使用 CUDA 设备或自定义模型：

```bash
# 第一块 CUDA GPU
roslaunch edge_yolo_ros yolo_detector.launch device:=0

# 自定义 Ultralytics 权重
roslaunch edge_yolo_ros yolo_detector.launch \
  weights:=/home/USER/models/best.pt device:=0
```

## 4. 查看结果

在已配置 ROS 网络并加载边缘侧工作空间的终端中执行：

```bash
rostopic list
rostopic echo /yolo/detections
rqt_image_view /yolo/annotated_image
```

主要话题如下：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/usb_cam/image_raw` | `sensor_msgs/Image` | 摄像头原始图像，仅在端侧使用 |
| `/uav/image_preprocessed/compressed` | `sensor_msgs/CompressedImage` | 跨设备传输的压缩图像 |
| `/yolo/detections` | `edge_yolo_ros/DetectionArray` | 检测框、类别、置信度及推理耗时 |
| `/yolo/annotated_image` | `sensor_msgs/Image` | 绘制检测框后的 BGR 图像 |

`DetectionArray` 包含原图尺寸、单帧推理耗时和检测列表；每个 `Detection` 包含 `class_id`、`class_name`、`confidence` 以及 `xmin/ymin/xmax/ymax`。

## 配置参数

端侧参数由 `edge_camera.launch` 提供：

| 参数 | 默认值 | 说明 |
| --- | ---: | --- |
| `video_device` | `/dev/video0` | 摄像头设备 |
| `camera_width` / `camera_height` | `640` / `480` | 原始采集尺寸 |
| `output_width` / `output_height` | `416` / `312` | 压缩前缩放尺寸 |
| `output_fps` | `10.0` | 最大输出帧率，必须大于 0 |
| `jpeg_quality` | `70` | JPEG 质量，范围 1–100 |

边缘侧默认参数位于 `edge/src/edge_yolo_ros/config/detector.yaml`：

| 参数 | 默认值 | 说明 |
| --- | ---: | --- |
| `confidence` | `0.40` | 置信度阈值 |
| `iou_threshold` | `0.45` | NMS IoU 阈值 |
| `image_size` | `640` | YOLO 推理输入尺寸 |
| `max_detections` | `100` | 单帧最大检测数 |
| `device` | `cpu` | 推理设备；launch 参数会覆盖此值 |
| `publish_annotated_image` | `true` | 是否发布标注图像 |

## 常见问题

- **边缘侧看不到端侧话题**：检查 `ROS_MASTER_URI`、两端 `ROS_IP`、ROS Master 是否启动，以及虚拟机是否处于桥接网络。
- **能看到话题但收不到图像**：确认两端可以双向访问，防火墙未阻止 ROS 动态端口，并避免把 `ROS_IP` 配成回环或 NAT 地址。
- **摄像头启动失败**：检查 `/dev/video*` 是否存在、当前用户是否有设备访问权限，以及 `video_device` 是否正确。
- **YOLO 节点导入失败**：在启动节点的同一虚拟环境中验证 `rospy`、`cv2`、`torch` 和 `ultralytics` 均可导入。若出现`ultralytics`无法导入的情况，
请手动指定相应python解释器重新编译工作空间。

## 第三方组件

仓库包含 ROS `usb_cam` 源码和 Ultralytics YOLO 权重。许可证与第三方声明见 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。
