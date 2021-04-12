#! /usr/bin/env python
"""
## =========================================================================== ## 
MIT License
Copyright (c) 2020 Roman Parak
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
## =========================================================================== ## 
Author   : Roman Parak
Email    : Roman.Parak@outlook.com
Github   : https://github.com/rparak
File Name: read_joint_pose.py
## =========================================================================== ## 
"""

# System (Default Lib.)
import sys

# Python client library for ROS
import rospy

# Package offers wrappers for the functionality provided in MoveIt
import moveit_commander

# Data types (messages, services)
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

# Numpy (Array computing Lib.)
import numpy as np

def main():
    # Node initialization
    rospy.init_node('read_joint_init', anonymous=True)

    group     = moveit_commander.MoveGroupCommander('manipulator')
    publisher = rospy.Publisher('current_joint_pose', numpy_msg(Floats), queue_size=5)

    try:
        while not rospy.is_shutdown():
            c_jP       = group.get_current_joint_values()
            fConv_c_jP = np.array([c_jP[0], c_jP[1], c_jP[2], c_jP[3], c_jP[4], c_jP[5]], dtype=np.float32) 
            publisher.publish(fConv_c_jP)

            # rospy rate sleep -> 1 Hz
            rospy.Rate(1).sleep()
    except:
        print('Unexpected error:', sys.exc_info()[0])
    finally:
        print('Bye.')

if __name__ == '__main__':
    sys.exit(main())