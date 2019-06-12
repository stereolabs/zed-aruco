# Stereolabs ZED - ArUco Positional Tracking sample

This sample shows how to reset the ZED camera tracking to a known reference using an ArUco marker.

![http://chev.me/arucogen/](../6x6_1000-25.png)

Point the camera to this marker and press the space bar to reset the positional tracking reference.
This feature can be useful to avoid accumulating drift when moving the camera or to get multiple cameras in the same referential.


## Getting started

- First, download the latest version of the ZED SDK on [stereolabs.com](https://www.stereolabs.com).
- For more information, read the ZED [API documentation](https://www.stereolabs.com/developers/documentation/API/).

### Prerequisites

- Windows 7 64bits or later, Ubuntu 16.04 or later
- [ZED SDK](https://www.stereolabs.com/developers/) and its dependencies ([CUDA](https://developer.nvidia.com/cuda-downloads))

## Build the program

Download the sample and follow the instructions below: [More](https://www.stereolabs.com/docs/getting-started/application-development/)

### Build for Windows

- Create a "build" folder in the source folder
- Open cmake-gui and select the source and build folders
- Generate the Visual Studio `Win64` solution
- Open the resulting solution and change configuration to `Release`
- Build solution

### Build for Linux

Open a terminal in the sample directory and execute the following command:

    mkdir build
    cd build
    cmake ..
    make

## Run the program

- Navigate to the build directory and launch the executable file
- Or open a terminal in the build directory and run the sample :

        ./ZED_Reloc_Aruco


## ArUco markers

This [website](http://chev.me/arucogen/) can be used to easily display Aruco patterns.

The sample is expecting a `6x6 Aruco` Dictionary with a 160mm marker by default. The bigger the marker, the better the camera position will be.
It's important to make sure that the real-world size of the marker matches the size set in the samples (160mm) to avoid scale issues in the tracking.

## ArUco detection

The Aruco detection code is taken from OpenCV Contrib module, please refer to the source files for the license information (BSD 3).
