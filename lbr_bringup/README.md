## 1. Description

This package bringups the interface and handles the communication with the KUKA iiwa.
There are two types of control interface:
- Socket Communication PTP
- FRI

To bringup the robot using socket communication, launch [lbr_bringup_ptp.launch.py](/launch/lbr_bringup_ptp.launch.py), which will launch three communication nodes: 
- lbr_commands_node
- lbr_statusdata_node
- lbr_sensordata_node

The connection type (UDP/TCP) can be set in the launch parameters. 
The IP address to your computer should be set in the parameter file [bringup_ptp.yaml](config/bringup_ptp.yaml). 
You can also change the port number for each of the nodes. 

To bring up the robot using FRI, launch the [lbr_bringup_fri.launch.py](/launch/lbr_bringup_fri.launch.py).

## 2. Run
To launch all of the communcation nodes, run: 
```bash
ros2 launch lbr_bringup lbr_bringup lbr_bringup_ptp.launch.py
# OR
ros2 launch lbr_bringup lbr_bringup lbr_bringup_fri.launch.py
```
