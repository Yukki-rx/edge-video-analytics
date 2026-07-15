# 双设备视频流边缘计算系统（ROS1 + YOLO）

本分支实现 baseline 视频分析链路：端侧设备 A 采集图像并完成缩放、限帧和 JPEG
压缩，通过 ROS 话题发送到边缘设备 B，由 YOLO 完成目标检测。

`master` 分支仅保留 baseline 链路；同时包含 baseline 与 improved 链路的开发版本位于
`dev` 分支。

## 数据流

~~~text
端侧设备 A                                  边缘设备 B

USB 摄像头
  -> /usb_cam/image_raw
  -> image_compressor
  -> /uav/image_preprocessed/compressed (ROS)
                                             -> yolo_detector
                                             -> /yolo/detections
                                             -> /yolo/annotated_image
                                             -> metrics_recorder
                                             -> results/
~~~

## 项目结构

~~~text
endpoint_device/                         端侧 catkin 工作空间
├─ start.sh                              编译并启动端侧 baseline
└─ src/
   ├─ robot_vision/
   │  ├─ launch/endpoint.launch         baseline 启动文件
   │  └─ scripts/image_compressor.py    缩放、限帧和 JPEG 压缩
   └─ usb_cam/                          V4L USB 摄像头驱动

edge_device/                             边缘侧 catkin 工作空间
├─ start.sh                              编译并启动边缘侧 baseline
└─ src/edge_yolo_ros/
   ├─ launch/edge.launch                baseline 启动文件
   ├─ scripts/yolo_detector.py          YOLO 检测
   ├─ scripts/metrics_recorder.py       指标记录
   ├─ config/detector.yaml              检测参数
   └─ weights/                          模型权重
~~~

## 1. 网络与时钟

两台设备需要位于可互通的局域网。以下配置假设 ROS Master 运行在端侧：

~~~text
端侧设备 A IP    192.168.0.100
边缘设备 B IP    192.168.0.118
ROS Master       http://192.168.0.100:11311
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

端侧需要 ROS1 Noetic、catkin、cv_bridge 和 V4L2。将 `endpoint_device` 作为
`~/endpoint_ws`，然后编译：

~~~bash
source /opt/ros/noetic/setup.bash
cd ~/endpoint_ws
catkin_init_workspace src
catkin_make
source devel/setup.bash
~~~

启动：

~~~bash
roslaunch robot_vision endpoint.launch
~~~

完成上述编译和网络环境配置后，也可以在仓库根目录直接运行
`./endpoint_device/start.sh`。脚本只加载 ROS 与已编译的工作空间环境，然后启动
baseline；可通过 `ROS_SETUP` 覆盖默认的 `/opt/ros/noetic/setup.bash`。

默认输出为 416×312、10 FPS、JPEG 质量 70，可按需调整：

~~~bash
roslaunch robot_vision endpoint.launch \
  output_width:=640 output_height:=480 output_fps:=10 jpeg_quality:=70
~~~

检查端侧输出：

~~~bash
rostopic hz /uav/image_preprocessed/compressed
~~~

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

将 `edge_device` 作为 `~/edge_ws`，然后编译：

~~~bash
cd ~/edge_ws
catkin_init_workspace src
catkin_make -DPYTHON_EXECUTABLE=$VIRTUAL_ENV/bin/python3
source devel/setup.bash
~~~

CPU 推理：

~~~bash
roslaunch edge_yolo_ros edge.launch device:=cpu
~~~

完成上述编译和网络环境配置后，也可以运行
`./edge_device/start.sh device:=cpu`。脚本只加载 ROS、已有 Python 虚拟环境和已编译的
工作空间，然后启动 baseline；默认使用 `~/venvs/edge_yolo`，可通过 `EDGE_VENV`
指定其他虚拟环境。

CUDA 推理使用 `device:=0`。自定义模型可通过
`weights:=/path/to/best.pt` 指定。

## 4. 参数与输出

检测参数位于 `edge_device/src/edge_yolo_ros/config/detector.yaml`：

| 参数 | 默认值 | 说明 |
| --- | ---: | --- |
| `confidence` | 0.40 | 最低检测置信度 |
| `iou_threshold` | 0.45 | NMS IoU 阈值 |
| `image_size` | 640 | 推理输入尺寸 |
| `max_detections` | 100 | 单帧最大检测数 |
| `device` | cpu | CPU 或 CUDA 设备编号 |
| `publish_annotated_image` | true | 是否发布带检测框图像 |

查看检测结果：

~~~bash
rostopic echo /yolo/detections
rqt_image_view /yolo/annotated_image
~~~

`metrics_recorder` 会在退出时将统计结果写入
`~/edge_yolo_ros/results/baseline_<时间戳>/`，其中包含 `summary.json` 和
`frames.csv`。

## 5. 常见问题

1. 看不到跨设备话题：检查 `ROS_MASTER_URI`、`ROS_IP`、防火墙和双向 ping。
2. 没有压缩图像：检查 USB 摄像头节点和 `/usb_cam/image_raw`。
3. YOLO 导入失败：确认编译与运行使用同一 Python 环境，并已安装
   `requirements.txt` 中的依赖。
4. 推理速度较慢：减小 `image_size`，或切换到 CUDA 设备。

## 许可证

- 仓库根目录的项目原创材料：MIT，见 [`LICENSE`](LICENSE)。
- `edge_yolo_ros`：GNU AGPL-3.0-only。
- `robot_vision`：MIT。
- `usb_cam`：BSD 3-Clause。
- Ultralytics 与模型权重遵循各自适用的许可条款。

第三方组件与许可路径见 `THIRD_PARTY_NOTICES.md`。
