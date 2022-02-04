# Rocket-Tracker

## Requirements


- OpenCV 4
- ROS melodic
- CUDA (10.2+ recommended), TensorRT 8.0+, CuDNN

In order to successfully compile and use this package, a vision_opencv package that was compiled against OpenCV 4 is required. For example, you can clone [this](https://github.com/DavidBaldsiefen/vision_opencv) into your `catkin_ws/src` folder.

## Installation

The entire software was tested on Ubuntu 18.04 as well as on an NVIDIA Jetson AGX Xavier with JetPack 4.6 (with CUDA, TensorRT and CuDNN dependencies preinstalled)

1. First make sure you have the following dependencies installed:
  - OpenCV4 (CUDA not required)
  - ROS Melodic Morenia
  - CUDA 10.2+
  - TensorRT 8.0+
  - CuDNN

2. Create a catkin workspace: http://wiki.ros.org/catkin/Tutorials/create_a_workspace
3. clone this repository into `catkin_ws/src`
4. clone the [modified cv_bridge](https://github.com/DavidBaldsiefen/vision_opencv) into `catkin_ws/src`. Any other vision_opencv package that was compiled against OpenCV4 should do too
5. run catkin_make inside of `catkin_ws`

## Usage

The software is launched witht the command `roslaunch rocket_tracker rocket_tracker.launch`. Alternatively, `headless_tracker.launch` contains the same functionality but does not spawn the `evaluator-node` containing the GUI.
The following launch arguments are available:

  - `video` path to the input video file
  - `weights` path to the TensorRT-Enginefile
  - `logtime` Set to true to enable additional time logging printouts to the console

## Information about the TensorRT engine

In general, any explicitly single batch TensorRT engine with 3-channel input bindings can be fed into the software. E.g. the input bindings could look like this: [1x3x640x640]. The software will automatically scale down the image if necessary. The software supports FP32 and FP16 engines, provided that the input and output channels are in FP32 precision.

However, the engine is currently only able to interpret output that equals the YOLO format. Specifically, this means that each detections is in the format [x, y, width, height, box_confidence, class_confidence_1, class_confidence_2, ....]. The model is able to identify the number of classes automatically.

For single-class engines, the software will always choose the detection with the highest box_confidence. For multi-class engines, the box_confidence will be multiplied with each class_confidence to get the final value.
