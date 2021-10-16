#!/usr/bin/env python3

# Copyright 2019 Nina Marie Wahl and Charlotte Heggem.
# Copyright 2019 Norwegian University of Science and Technology.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# 
# Modified by Joshua Liu (jsliu@jhu.edu) on 2021-9-25

import rclpy
import sys, select, termios, tty
from sensor_msgs.msg import JointState

import math

msg = """
Reading from the keyboard!
---------------------------
Moving joints in positive direction:
1 2 3 4 5 6 7
For moving in negative direction, hold down the shift key:
---------------------------
! @ # $ % ^ &

Press "0" to home all joints

q/z : increase/decrease speed by 10%
CTRL-C to quit
"""

moveBindings = {
        '1':("a1",1),
        '2':("a2",1),
        '3':("a3",1),
        '4':("a4",1),
        '5':("a5",1),
        '6':("a6",1),
        '7':("a7",1),
        '!':("a1",-1),
        '@':("a2",-1),
        '#':("a3",-1),
        '$':("a4",-1),
        '%':("a5",-1),
        '^':("a6",-1),
        '&':("a7",-1),
        '0':("a0",0),
    }

speedBindings={
        'q':1.1,
        'z':.9,
    }

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    print(key)
    return key

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = rclpy.create_node('keyboard_teleop')
    pub = node.create_publisher(JointState, '/joint_states', 10)

    speed = 0.1 * math.pi / 180

    joint_msg = JointState()
    joint_msg.name.append("joint_a1")
    joint_msg.name.append("joint_a2")
    joint_msg.name.append("joint_a3")
    joint_msg.name.append("joint_a4")
    joint_msg.name.append("joint_a5")
    joint_msg.name.append("joint_a6")
    joint_msg.name.append("joint_a7")

    # Home joint values
    joint_a1 = -1.5810079050729513
    joint_a2 = 0.8167376905451679
    joint_a3 = -0.004375380628040148
    joint_a4 = -1.5857193233230227
    joint_a5 = -7.76584250674877e-06
    joint_a6 = 0.7069321699035735
    joint_a7 = -0.8036188590817025

    joint_msg.position.append(joint_a1)
    joint_msg.position.append(joint_a2)
    joint_msg.position.append(joint_a3)
    joint_msg.position.append(joint_a4)
    joint_msg.position.append(joint_a5)
    joint_msg.position.append(joint_a6)
    joint_msg.position.append(joint_a7)

    status=0

    try:
        print(msg)
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                joint = moveBindings[key][0]
                if joint == "a1":
                    joint_msg.position[0] = joint_msg.position[0] + moveBindings[key][1] * speed
                elif joint == "a2":
                    joint_msg.position[1] = joint_msg.position[1] + moveBindings[key][1] * speed
                elif joint == "a3":
                    joint_msg.position[2] = joint_msg.position[2] + moveBindings[key][1] * speed
                elif joint == "a4":
                    joint_msg.position[3] = joint_msg.position[3] + moveBindings[key][1] * speed
                elif joint == "a5":
                    joint_msg.position[4] = joint_msg.position[4] + moveBindings[key][1] * speed
                elif joint == "a6":
                    joint_msg.position[5] = joint_msg.position[5] + moveBindings[key][1] * speed
                elif joint == "a7":
                    joint_msg.position[6] = joint_msg.position[6] + moveBindings[key][1] * speed
                elif joint == "a0":
                    joint_msg.position[0] = joint_a1
                    joint_msg.position[1] = joint_a2
                    joint_msg.position[2] = joint_a3
                    joint_msg.position[3] = joint_a4
                    joint_msg.position[4] = joint_a5
                    joint_msg.position[5] = joint_a6
                    joint_msg.position[6] = joint_a7

                # time stamp
                joint_msg.header.stamp = node.get_clock().now().to_msg()
                pub.publish(joint_msg)

            elif key in speedBindings.keys():
                speed = speed * speedBindings[key]

            else:
                if (key == '\x03'):
                    break

            if (status == 14):
                print(msg)
            status = (status + 1) % 15

    except Exception as e:
        print(e)

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
