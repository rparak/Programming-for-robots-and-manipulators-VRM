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
from std_srvs.srv import Empty 
from turtlesim.srv import SetPen, TeleportAbsolute

def move(lin_vel_x, ang_vel_th):
    """
    Description:
        TurtleSim robot velocity control function. A simple demonstration of working with parameters, services, etc.

        More information about Turtlesim at:
        http://wiki.ros.org/turtlesim
    Args:
        (1) lin_vel_x [Float]: Linear Velocity (x).
        (2) ang_vel_th [Float]: Angular Velocity (theta).
    """

    # Turtlesim Node initialization (new)
    rospy.init_node('turtle_init', anonymous=True)

    # Publisher initialization
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)

    # Set background Color 
    rospy.set_param('/sim/background_r', 0)
    rospy.set_param('/sim/background_g', 255)
    rospy.set_param('/sim/background_b', 75)

    # Get the currently used Ros Distribution
    ros_d = rospy.get_param('/rosdistro')
    rospy.loginfo('ROS Distribution: %s', ros_d)

    # Reset Environment
    rospy.wait_for_service('/reset')
    reset_simulator   = rospy.ServiceProxy('/reset', Empty)
    reset_simulator()

    # Clear Environment
    rospy.wait_for_service('/clear')
    clear_simulator   = rospy.ServiceProxy('/clear', Empty)

    # Turtle Teleport (Absolute)
    rospy.wait_for_service('/turtle1/teleport_absolute')
    turtle_teleport = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)

    # Pen Color
    rospy.wait_for_service('/turtle1/set_pen')
    set_pen_simulator = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
    
    # Call Functions
    turtle_teleport(5, 5, 0)
    set_pen_simulator(250,0,0,5,0)
    clear_simulator()

    # Set the Rate (10hz)
    rate = rospy.Rate(10)

    vel_msg = Twist()

    # Set the parameters of the turtle movement
    vel_msg.linear.x  = lin_vel_x
    vel_msg.linear.y  = 0
    vel_msg.linear.z  = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = ang_vel_th

    while not rospy.is_shutdown():
        # Move (Publish parameters)
        velocity_publisher.publish(vel_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        # Read Parameters from Argument: rosrun t_ctrl test.py 2.0 1.0
        move(float(sys.argv[1]), float(sys.argv[2]))
    except rospy.ROSInterruptException: 
        pass
