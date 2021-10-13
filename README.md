# iiwa_ros2

A ROS2 package for controlling KUKA iiwa


## Getting Started

1. [Install ROS2 Rolling](https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debians.html) or [Install ROS2 Galactic](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html). This branch will support both distributions until API breaking changes are made, at which point a `galactic` branch will be forked. 

2. Make sure that `colcon`, its extensions and `vcs` are installed:
   ```
   sudo apt install python3-colcon-common-extensions python3-vcstool
   ```

3. Create a new ROS2 workspace:
   ```
   export COLCON_WS=~/kuka_ws
   mkdir -p $COLCON_WS/src
   ```

4. Pull relevant packages, install dependencies, compile, and source the workspace by using:
   ```
   cd $COLCON_WS/src
   git clone https://github.com/jhu-bigss/iiwa_ros2.git
   cd ..
   vcs import src --skip-existing --input src/iiwa_ros2/Dependencies.repos
   rosdep install --ignore-src --from-paths src -y -r
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
   source install/setup.bash
   ```

## Using MoveIt

To use MoveIt some additional packages should be added into workspace:
   ```
   cd $COLCON_WS
   vcs import src --skip-existing --input src/Universal_Robots_ROS2_Driver/MoveIt_Support.repos
   vcs import src --skip-existing --input src/moveit2/moveit2.repos
   rosdep install --ignore-src --from-paths src -y -r
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
   source install/setup.bash
   ```

## Dependencies

* [realtime_tools](https://github.com/ros-controls/realtime_tools/tree/foxy-devel)
* [hardware_interface](https://github.com/ros-controls/ros2_control/tree/master/hardware_interface)

```bash
sudo apt install ros-<distro>-realtime-tools ros-<distro>-hardware-interface
```

## Documentation

- Use [iiwa_bringup](/iiwa_bringup/) to launch the KUKA iiwa robot visulization in Rviz.

## Reference

1. [kmriiwa_ws](https://github.com/ninamwa/kmriiwa_ws)
2. [iiwa_ros](https://github.com/epfl-lasa/iiwa_ros)
