#!/usr/bin/env python
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
File Name: bezier_curve_example_2d.py
## =========================================================================== ## 
"""

# System (Default Lib.)
import sys

# Python client library for ROS
import rospy

# Data types (messages, services)
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_srvs.srv import Empty 
from turtlesim.srv import SetPen, TeleportAbsolute

def pose_callback(pose):
    """
    Description:
        Function to obtain a string with the number of points used in the calculation of the curve. (Figure Legend -> Label Name)
        http://wiki.ros.org/turtlesim
    Args:
        (1) s_index [INT]: Number of points for calculation.
    Returns:
        (1) param 1 [String]: The resulting string for the label.
    """

    rospy.loginfo("Turtlesim X = %f, Y = %f, TH = %f", pose.x, pose.y, pose.theta)

def read_pos():
    rospy.init_node('turtle_read', anonymous=True)
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        read_pos()
    except rospy.ROSInterruptException: 
        pass
