# edge_yolo_ros

运行在边缘设备 B 上的 ROS1 YOLO 推理包，支持两种模式：

- `baseline`：订阅 JPEG 压缩图像并执行 YOLO 检测。
- `improved`：接收 H.264 RTP 流，执行 YOLO + ByteTrack，并向端侧反馈帧率和码率。

## 编译与运行

~~~bash
source /opt/ros/noetic/setup.bash
source ~/venvs/edge_yolo/bin/activate
cd ~/edge_ws
catkin_init_workspace src
catkin_make -DPYTHON_EXECUTABLE=$VIRTUAL_ENV/bin/python3
source devel/setup.bash

roslaunch edge_yolo_ros edge.launch mode:=baseline device:=cpu
roslaunch edge_yolo_ros edge.launch mode:=improved device:=cpu
~~~

CUDA 推理使用 `device:=0`，自定义模型使用
`weights:=/path/to/best.pt`。

## ROS 接口

| 模式 | 方向 | 话题 | 消息类型 |
| --- | --- | --- | --- |
| baseline | 输入 | `/uav/image_preprocessed/compressed` | `sensor_msgs/CompressedImage` |
| improved | 输入 | `/uav/image_decoded` | `sensor_msgs/Image` |
| improved | 输出 | `/uav/stream_control` | `stream_msgs/StreamControl` |
| 两者 | 输出 | `/yolo/detections` | `edge_yolo_ros/DetectionArray` |
| improved | 输出 | `/yolo/tracks` | `edge_yolo_ros/TrackArray` |
| 两者 | 输出 | `/yolo/annotated_image` | `sensor_msgs/Image` |

检测参数位于 `config/detector.yaml`。统一启动参数位于 `launch/edge.launch`。

## 许可证

本包采用 GNU AGPL-3.0-only。Ultralytics 软件和模型权重遵循各自适用的许可条款。
