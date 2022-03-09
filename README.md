# Rocket-Tracker

This repository contains a [ROS](https://www.ros.org/) implementation of a rocket tracker capable of detecting rockets at high speeds using neural networks.

The software may also be used to solve similar detection problems, or as groundwork for general object detection using YOLOv5 or YOLOX in C++ and ROS.

There are two up-to-date branches:

- The [TensorRT-NoSharedMemory](https://github.com/DavidBaldsiefen/rocket_tracker/tree/TensorRT-NoSharedMemory) branch contains the codebase for a basic image processor which relies entirely on the ROS publisher/subscriber pipeline
- The [TensorRT-SharedMemory](https://github.com/DavidBaldsiefen/rocket_tracker/tree/TensorRT-SharedMemory) branch which replaces the ROS publisher/subscriber pipeline with a shared memory segment, allowing for lower latencies and higher throughputs.

Both branches are otherwise identical. Installation instructions can be found in the READMEs of each branch.

### Performance Measurements on an NVIDIA Jetson AGX Xavier

![Performance Measurements](https://github.com/DavidBaldsiefen/rocket_tracker/blob/main/thesis-results.png)
### Reproducing the Performance Measurements

To reproduce these results, check out and install one of the two branches above.

Afterwards, download the [weights](https://github.com/DavidBaldsiefen/rocket_tracker/releases/download/v0.1/rocket-weights.pt) and [test-video](https://github.com/DavidBaldsiefen/rocket_tracker/releases/download/v0.1/rocket-video.mp4) which were used in the performance measurements. To convert the weights to TensorRT, clone the official [YOLOv5 repository](https://github.com/ultralytics/yolov5) and use their `export.py`:

    python export.py --weights path/to/rocket-weights.pt --include engine --device 0

Afterwards, make sure that the NVIDIA Jetson runs in `MAXN` power mode with `jetson_clocks` enabled, and then initiate the performance test using the following command:

`roslaunch rocket_tracker headless_tracker.launch weights:="path/to/rocket-weights.engine" video:="path/to/test-video.mp4" perftest:=True`

Performance measurements will be printed to the console every 1000 frames. The results above were obtained based on the 3rd console printout.
