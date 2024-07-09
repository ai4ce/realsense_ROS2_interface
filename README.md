# realsense_ROS2_interface

This ROS2 package interfaces robots and/or joysticks with the official Intel RealSense ROS wrapper.

While it's targeting Intel RealSense and Sony Playstation Joystick, the package can be easily modified to work with any USB camera and controller.

For any questions, please contact the author, [Irving Fang](https://irvingf7.github.io/).

This package is heavily influenced and inspired by a [similar ROS1 package](https://github.com/ripl/capture-images) written by [Dr. Takuma Yoneda](https://takuma.yoneda.xyz/) from TTIC.

## Setup
This package is only tested on `Ubuntu 22.04` and `ROS humble` distribution.

It requires the official [RealSense ROS2 wrapper](https://github.com/IntelRealSense/realsense-ros) and the Intel RealSense SDK.

I expect this package to be easily portable to other Ubuntu versions and ROS 2 distros, since it does not utilize anything specific to `Ubuntu 22.04` and `ROS humble`.



## Installation

After acquiring all the prerequisites, simply put this folder (which contains two ROS2 packages, technically speaking) inside your ROS workspace's `src` folder and run the following command in your ROS workspace.

```
colcon build
```


## Package Breakdown
I will walk through the package structure briefly here. Feel free to skip over if you are familiar with how a typical ROS2 package works.

This project is actually made of two ROS2 packages:

- `realsense_capture`: contains all the code that actually does the job.

- `realsense_interface_msg`: contains the definition for the service message type that's being used.

It's a conventional ROS practice to dedicate an independent package for message type definitions.

Inside `realsense_capture`:
- `realsense_image_server.py`: responsible for constantly getting images from the camera, and is ready to serve the image whenever requested, that is, the capture button on the joystick is hit.

- `realsense_image_client.py`: responsible for acquiring the image whenever the capture button on the joystick is hit. 


## Usage
To use the package, run

```bash
ros2 launch realsense_capture realsense_sys_launch.py
```

***Note***: The launch file also launches Foxglove, a visualization/monitoring app for ROS2. Get rid of related lines if desired. 

There is no need to launch the RealSense ROS wrapper separately, as they are integrated into the launch file.

### Parameters
There are many parameters you can tune that come with RealSense, but some of the most important ones are:

1. `camera_namespace` and `camera_name`: the topic name of your camera in ROS2 will be in the form of `camera_namespace/camera_name/xxx

2. `save_folder`: indicates where to save the `.png` of the depth and color images.


## Contributing

Contributions to this project are welcome. If you encounter any issues or have suggestions for improvements, please open an issue on the GitHub repository.

## Citation
If you find this little package helpful, please consider citing it in your paper/work.
