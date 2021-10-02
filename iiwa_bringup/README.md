## 1. Description

This package handles the different files for bringing up the robot and showing it in Rviz.

## 2. Requirements

The following packages needs to be installed:

- joint_state_publisher
- robot_state_publisher
```bash
$ sudo apt install ros-<distro>-joint-state-publisher ros-<distro>-robot-state-publisher
```
- joint_state_publisher_gui (optional)
```bash
$ sudo apt install ros-<distro>-joint-state-publisher-gui
```

## 3. Run

To visualize the URDF model of the robot in Rviz, you need two terminals and run the following commands:

```
$ ros2 launch iiwa_bringup rviz.launch.py
```

```
$ ros2 run iiwa_bringup keyboard_teleop.py
```
OR
```
$ ros2 run iiwa_bringup dummy_joint_states
```
The latter will run a dummy joint state publisher which publishes fake data for the joints which are not fixed. This is necessary to properly visualize the manipulator.