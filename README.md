# Config4

This specific configuration consists of a Raspberry Pi 4 with the Pi Camera Module v3. The Raspberry Pi 4 serves as the platform for image processing. This is quite similar to config 2, but without the TPU.

In this configuration, OpenCV Classic is employed with an emphasis on color detection for image processing. Color detection is a fundamental technique in computer vision that allows for the identification and tracking of objects based on their specific color signatures.
This image processing technique does computation on the pixels from the camera, and therefor require less computational power from the CPU as a trained Tensorflow/YOLO model would. And is therefore the reason why we are running OpenCV Classic on this configuration, due to no TPU.

## Table of Contents

- [Prerequisites](#prerequisites)
- [Installation](#installation)
  - [Option 1: Manual Installation](#option-1-manual-installation)
  - [Option 2: Using Docker](#option-2-using-docker)
- [Configuration](#configuration)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)
- [Contact](#contact)

## Prerequisites

### OS

- Raspberry Pi OS - Bullseye (2023-02-21)

Make sure you use the latest version of the Raspberry Pi OS. You can download the latest version from the official Raspberry Pi website: https://www.raspberrypi.org/software/operating-systems/

The reason for choosing the Raspberry Pi OS is due to the newly released Camera Module v3. The camera module v3 is not supported by the Ubuntu 22.04, and therefore we have to use the Raspberry Pi OS.

### Hardware

- Raspberry Pi 4
- Raspberry Pi Camera Module v3
- Micro SD Card (16GB or larger)

### Software

Dependencies will be under the installation section.


## Installation

We have 2 difference methods of installation. You can either install the dependencies manually, or you can use Docker. Both should work fine, but we recommend using Docker.

### Option 1: Manual Installation

1. Attach the Raspberry Pi Camera Module v3 to the Raspberry Pi 4.
2. Update the Raspberry Pi OS to the latest version:
    Make sure that you have the Pi Camera Module v3 attached to the Raspberry Pi 4 before updating the OS. (So that you download the drivers for the camera module)
3. Install Ros2 Humble through this [link](https://github.com/Ar-Ray-code/rpi-bullseye-ros2.git)
4. 

    

    

