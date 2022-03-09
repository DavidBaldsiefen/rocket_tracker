# Rocket-Tracker

This repository contains a [ROS](https://www.ros.org/) implementation of a rocket tracker capable of detecting rockets at high speeds using neural networks.

The software may also be used to solve similar detection problems, or as groundwork for general object detection using YOLOv5 or YOLOX in C++ and ROS.

There are two up-to-date branches:

- The [TensorRT-NoSharedMemory](https://github.com/DavidBaldsiefen/rocket_tracker/tree/TensorRT-NoSharedMemory) branch contains the codebase for a basic image processor which relies entirely on the ROS publisher/subscriber pipeline
- The [TensorRT-SharedMemory](https://github.com/DavidBaldsiefen/rocket_tracker/tree/TensorRT-SharedMemory) branch which replaces the ROS publisher/subscriber pipeline with a shared memory segment, allowing for lower latencies and higher throughputs.

Both branches are otherwise identical. Installation instructions can be found in the READMEs of each branch.

### Performance Measurements on an NVIDIA Jetson AGX Xavier

![Performance Measurements](https://github.com/DavidBaldsiefen/rocket_tracker/blob/main/thesis-results.png)
