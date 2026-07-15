# edge_yolo_ros

运行在边缘设备 B 上的 ROS1 YOLO 推理包。节点接收端侧设备 A 发布的压缩图像，输出结构化检测结果和带检测框的图像。

## ROS 接口

| 方向 | 话题 | 消息类型 |
| --- | --- | --- |
| 输入 | /uav/image_preprocessed/compressed | sensor_msgs/CompressedImage |
| 输出 | /yolo/detections | edge_yolo_ros/DetectionArray |
| 输出 | /yolo/annotated_image | sensor_msgs/Image |

检测消息包含类别 ID、类别名称、置信度、检测框坐标、图像尺寸和推理耗时。

## 环境

推荐 Ubuntu 20.04、ROS Noetic 和 Python 3。先安装适合 CPU 或 CUDA 环境的
PyTorch，再安装 Ultralytics：

~~~bash
source /opt/ros/noetic/setup.bash
python3 -m venv --system-site-packages ~/venvs/edge_yolo
source ~/venvs/edge_yolo/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
python3 -m pip install -r ~/edge_ws/src/edge_yolo_ros/requirements.txt
python3 -c "import rospy, torch, ultralytics; print('environment OK')"
~~~

## 编译

~~~bash
source /opt/ros/noetic/setup.bash
source ~/venvs/edge_yolo/bin/activate
chmod +x ~/edge_ws/src/edge_yolo_ros/scripts/yolo_detector.py
cd ~/edge_ws
catkin_init_workspace src
catkin_make -DPYTHON_EXECUTABLE=$VIRTUAL_ENV/bin/python3
source devel/setup.bash
~~~

## 运行

CPU：

~~~bash
source /opt/ros/noetic/setup.bash
source ~/venvs/edge_yolo/bin/activate
source ~/edge_ws/devel/setup.bash
roslaunch edge_yolo_ros yolo_detector.launch device:=cpu
~~~

CUDA：

~~~bash
roslaunch edge_yolo_ros yolo_detector.launch device:=0
~~~

指定模型：

~~~bash
roslaunch edge_yolo_ros yolo_detector.launch \
  weights:=/home/USER/models/best.pt device:=cpu
~~~

查看结果：

~~~bash
rostopic echo /yolo/detections
rqt_image_view /yolo/annotated_image
~~~

## 配置

默认配置位于 **config/detector.yaml**：

| 参数 | 默认值 | 说明 |
| --- | ---: | --- |
| confidence | 0.40 | 最低检测置信度 |
| iou_threshold | 0.45 | NMS IoU 阈值 |
| image_size | 640 | 推理输入尺寸 |
| max_detections | 100 | 单帧最大检测数 |
| device | cpu | CPU 或 CUDA 设备编号 |
| publish_annotated_image | true | 是否发布带框图像 |

启动文件默认加载 **weights/yolov8n.pt**。仓库还包含 **weights/yolo26n.pt**；
自定义任务可通过 weights 参数加载自己的 best.pt。

## 许可证

本包直接集成 Ultralytics YOLO，因此采用 **GNU AGPL-3.0-only**，完整条款见
**LICENSE**。Ultralytics 软件和模型权重仍受其上游许可证约束；需要闭源或
不适用 AGPL 的商业部署时，应取得相应商业授权。
