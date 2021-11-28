# Rocket-Tracker

## Requirements

- OpenCV 4
- ROS melodic
- libtorch

In order to successfully compile and use this package, a vision_opencv package that was compiled against OpenCV 4 is required. For example, you can clone [this](https://github.com/DavidBaldsiefen/vision_opencv) into your `catkin_ws/src` folder. In addition, install [libtorch](https://pytorch.org/get-started/locally/) by extracting the `libtorch` folder to the `catkin_ws/src` folder (or anywhere else, provided you update the `TORCH_DIR` parameter in `CMakeLists.txt` of the core-package
