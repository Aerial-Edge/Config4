# Config4

This specific configuration consists of a Raspberry Pi 4 with the Pi Camera Module v3. The Raspberry Pi 4 serves as the platform for image processing. This is quite similar to config 2, but without the TPU.

In this configuration, OpenCV Classic is employed with an emphasis on color detection for image processing. Color detection is a fundamental technique in computer vision that allows for the identification and tracking of objects based on their specific color signatures.
This image processing technique does computation on the pixels from the camera, and therefor require less computational power from the CPU as a trained Tensorflow/YOLO model would. And is therefore the reason why we are running OpenCV Classic on this configuration, due to no TPU.

## Table of Contents

- [Prerequisites](#prerequisites)
- [Installation](#installation)
  - [Option 1: Manual Installation](#option-1-manual-installation)
  - [Option 2: Using Docker](#option-2-using-docker)
- [Software Reasoning](#Software-Reasoning)
- [References](#references)

## Prerequisites

### OS

- Raspberry Pi OS - Bullseye 64bit (2023-02-21)

Make sure you use the latest version of the Raspberry Pi OS. You can download the latest version from the official Raspberry Pi website: https://www.raspberrypi.org/software/operating-systems/

The reason for choosing the Raspberry Pi OS is due to the newly released Camera Module v3. The camera module v3 is not supported by the Ubuntu 22.04, and therefore we have to use the Raspberry Pi OS.

### Hardware

- Raspberry Pi 4
- Raspberry Pi Camera Module v3
- Micro SD Card (16GB or larger)

### Software

- Docker (Optional)
- Python 3.8 or newer
- OpenCV 4.7.0
- ROS2 Humble
- VS Code (Optional)


## Installation

We have 2 difference methods of installation. You can either install the dependencies manually, or you can use Docker. Both should work fine, but we recommend using Docker.

### Option 1: Manual Installation

1. Attach the Raspberry Pi Camera Module v3 to the Raspberry Pi 4.
2. Update the Raspberry Pi OS to the latest version:
    Make sure that you have the Pi Camera Module v3 attached to the Raspberry Pi 4 before updating the OS. (So that you download the drivers for the camera module)
3. Install Ros2 Humble through this repo [link](https://github.com/Ar-Ray-code/rpi-bullseye-ros2.git)
4. Install VS Code (Optional):

   Download the latest version of VS Code from the official website: https://code.visualstudio.com/sha/download?build=stable&os=linux-deb-arm64
   ```sh	
    cd ~/Downloads
    sudo dpkg -i the-downloaded-file.deb
    ```

4. Create your workspace:

    ```sh
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws
    colcon build
    source install/setup.bash
    ``` 

5. Install required librarys:

    ```sh
    pip install opencv-contrib-python # OpenCV
    pip install cvzone # OpenCV helper library
    pip install numpy # Numpy
    pip install -U colcon-common-extensions # Colcon build tool for ROS2

    cd ~/ros2_ws/src
    git clone https://github.com/ros-perception/vision_opencv.git --branch humble # OpenCV bridge for ROS2
    cd ..
    colcon build --packages-select vision_opencv
    ``` 


6. Clone the config4 repository:

    ```sh
    cd ~/ros2_ws/src
    git clone https://github.com/Aerial-Edge/Config4.git --branch master
    ```
    
7. Build the workspace:

    ```sh
    cd ~/ros2_ws
    colcon build --symlink-install
    source install/setup.bash
    ```

8. Run the ros2 nodes:

    ```sh
    ros2 run config4 camera_capture
    ros2 run config4 object_detection
    ```

### Option 2: Using Docker

1. Attach the Raspberry Pi Camera Module v3 to the Raspberry Pi 4.
    

## Software Reasoning

### OpenCV

We have not tried other alternatives to OpenCV due how well it works. OpenCV is a very well documented library, and is very easy to use. It is also very fast, and can run on the Raspberry Pi 4 without any problems. OpenCV is also very well supported by the ROS2 community, and therefor it is easy to integrate with ROS2.

### ROS2

ROS2 was chosen due to wishes from our customer, and because it is a very well documented and supported framework. ROS2 is also very easy to use, and is very well supported by the community. ROS2 is also easy to integrate with OpenCV, and is therefor a good choice for this configuration.

### cvzone

cvzone is a helper library for OpenCV. We chose to use this library due to how easy it made it to draw bounding boxes around the objects we detected. It also made it simpler to draw text on the screen, and to draw the FPS counter. It is also very well documented, and is easy to use.

### Python

We chose to use Python due to how easy it is to use, and how well it works with OpenCV and ROS2.


## References

- [OpenCV](https://opencv.org/)
- [ROS2](https://docs.ros.org/en/humble/index.html)
- [cvzone](https://github.com/cvzone/cvzone)
- [VS Code](https://code.visualstudio.com/)
