# iiwa_ros2

A ROS2 package for controlling KUKA iiwa

## Packages in the Repository:

* `iiwa_bringup` - launch file to bring up the robot in Rviz.

## Getting Started

1. [Install ROS2 Rolling](https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debians.html) or [Install ROS2 Galactic](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html). This branch will support both distributions until API breaking changes are made, at which point a `galactic` branch will be forked. If you want to run the package in simulation, you need to install [gazebo-ros2-control](https://github.com/ros-simulation/gazebo_ros2_control):

   ```bash
   sudo apt-get install ros-galactic-ros2-control ros-galactic-ros2-controllers ros-galactic-gazebo-ros2-control
   ```
2. Make sure that `colcon`, its extensions and `vcs` are installed:

   ```bash
   sudo apt install python3-colcon-common-extensions python3-vcstool
   ```
3. Create your ROS2 workspace:

   ```bash
   export COLCON_WS=~/kuka_ws
   mkdir -p $COLCON_WS/src
   ```
4. Pull relevant packages, install dependencies, compile, and source the workspace by using:

   ```bash
   cd $COLCON_WS/src
   git clone --recurse-submodules https://github.com/jhu-bigss/kuka_lbr_ros2.git
   cd ..
   # Make sure all ROS-related dependencies are installed
   rosdep install --ignore-src --from-paths src -y -r
   # Build
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
   source install/setup.bash
   ```
5. To check that robot descriptions are working properly use following launch commands:

   ```bash
   ros2 launch lbr_description view_robot.launch.py # or view_robot_bigss.launch.py
   ```
6. To start the robot controller, open a terminal, source your ROS2-workspace and execute the launch file with:

   ```bash
   ros2 launch lbr_bringup lbr_bringup.launch.py model:=iiw7 sim:=true # model:=[iiwa7/iiwa14/med7/med14]
   ```

   For execution on the real robot, the steps in Real Setup are to be followed. Once the controller is set up, run the LBRServer app on the KUKA smartPAD. Once running, establish a connection via

   ```bash
   ros2 launch lbr_bringup lbr_bringup.launch.py model:=iiwa7 sim:=false # model:=[iiwa7/iiwa14/med7/med14]
   ```

## Using MoveIt

To use MoveIt2:

```bash
sudo apt install ros-galactic-moveit
```

## Dependencies

* [realtime_tools](https://github.com/ros-controls/realtime_tools/tree/foxy-devel)
* [hardware_interface](https://github.com/ros-controls/ros2_control/tree/master/hardware_interface)

## Documentation

- Use [lbr_bringup](/lbr_bringup/) to launch the KUKA iiwa robot visulization in Rviz.

## Reference

1. [kmriiwa_ws](https://github.com/ninamwa/kmriiwa_ws)
2. [lbr_fri_ros2_stack](https://github.com/KCL-BMEIS/lbr_fri_ros2_stack)
