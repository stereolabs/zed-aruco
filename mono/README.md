# Stereolabs ZED - ArUco Positional Tracking sample

This sample shows how to reset the ZED camera tracking to a known reference using an ArUco marker.

Point the camera to this marker and press the space bar to reset the positional tracking reference.
This feature can be useful to avoid accumulating drift when moving the camera or to get multiple cameras in the same referential.


## Getting started

- First, download the latest version of the ZED SDK on [stereolabs.com](https://www.stereolabs.com).
- For more information, read the ZED [API documentation](https://www.stereolabs.com/developers/documentation/API/).

### Prerequisites

- [ZED SDK](https://www.stereolabs.com/developers/) and its dependencies ([CUDA](https://developer.nvidia.com/cuda-downloads))
- A [Stereolabs Stereo Camera](https://www.stereolabs.com/)

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

        ./ZED_Reloc_Aruco [path/to/svo] [path/to/aruco_poses.txt]

## Key Bindings

- `p`: pause the video
- `space`: reset the camera position based on the closest aruco tag
- `z`: display the world coordinate system axes in the RGB image from the zed positional tracking
- `a`: display the world coordinate system axes in the RGB image from the aruco detection
- `d`: toggle the display of the positions of the camera pose estimated by aruco detection

## ArUco markers

This [website](http://chev.me/arucogen/) can be used to easily display Aruco patterns.

The sample is expecting a `6x6 Aruco` Dictionary with a 160mm marker by default. The bigger the marker, the better the camera position will be.
It's important to make sure that the real-world size of the marker matches the size set in the samples (160mm) to avoid scale issues in the tracking.

## ArUco detection

The Aruco detection code is taken from OpenCV Contrib module, please refer to the source files for the license information (BSD 3).
