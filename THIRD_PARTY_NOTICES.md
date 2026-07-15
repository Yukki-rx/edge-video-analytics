# Third-party notices

This repository is a multi-license collection. A license assigned to one package
does not override the license or copyright of another package.

Project-authored source code in `endpoint/src/robot_vision`, together with
`edge/src/edge_yolo_ros/scripts/yolo_detector.py` and other project-authored
integration files, is available under the MIT License. The third-party
components and resources listed below are excluded from that grant.

## usb_cam

- Path: endpoint/src/usb_cam
- Version: 0.3.6
- Upstream: https://github.com/bosch-ros-pkg/usb_cam
- License: BSD 3-Clause

The original LICENSE, AUTHORS.md, CHANGELOG.rst, src/LICENSE, and source-file
copyright notices are retained with the package.

## Ultralytics YOLO and model weights

- Integration path: edge/src/edge_yolo_ros
- Python dependency: ultralytics
- Upstream: https://github.com/ultralytics/ultralytics
- Official licensing information: https://www.ultralytics.com/license
- Upstream licensing: AGPL-3.0 or a separately obtained Ultralytics license

The project-authored ROS integration code is licensed under MIT, but the MIT
License does not relicense Ultralytics or any model weights. Installing,
distributing, or using those components remains subject to the terms supplied
by their respective authors and distributors. Ultralytics states that its
software and trained models are AGPL-3.0 by default, and that use without the
AGPL source-disclosure obligations requires an appropriate Ultralytics license.
Do not assume that the package license transfers ownership of a model or its
training data. This notice is not legal advice.

## OpenCV Haar cascade data

- Path: endpoint/src/robot_vision/data/haar_detectors

The XML files contain their original Intel/OpenCV copyright and redistribution
terms. Those embedded notices must be retained.
