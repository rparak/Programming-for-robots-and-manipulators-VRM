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
File Name: reset_robot.py
## =========================================================================== ## 
"""

# System (Default Lib.)
import sys

# Python client library for ROS
import rospy

# Package offers wrappers for the functionality provided in MoveIt
import moveit_commander

# Data types (messages, services)
from std_srvs.srv import Trigger

# Numpy (Array computing Lib.)
import numpy as np

class reset_robot_handler():
    """
    Description:
        The class displays a simple service command to reset the robot's positions (joint space).
    """

    def __init__(self):
        # Node initialization
        rospy.init_node('reset_init')   

        self.robotics_arm_name = 'manipulator'
        self.robot = moveit_commander.RobotCommander() 
        self.group = None

        while self.group == None:
            try:
                self.group = moveit_commander.MoveGroupCommander(self.robotics_arm_name)
            except Exception:
                rospy.sleep(1)

        # Trigger -> reset robot position (joints)
        self.set_trigger = rospy.Service('reset_robot', Trigger, self.reset_robot_pose)    

    def reset_robot_pose(self, request):
        # Get the robot's home initialization parameters from the robot.yaml
        joint_target = rospy.get_param('/robot_home_param')

        # Setting parameters for planning
        self.group.set_max_velocity_scaling_factor(0.25)
        self.group.set_max_acceleration_scaling_factor(0.25)

         # Set the parameters of the Robot (Joint Control)
        self.group.set_joint_value_target(joint_target)

        # Get a robot trajectory plan (Joint Control)
        plan = self.group.plan(joint_target)

        # Execute the trajectory
        self.group.execute(plan, wait=True)
        rospy.sleep(0.5)

        # Reset
        self.group.stop()
        self.group.clear_path_constraints()
        self.group.clear_pose_targets()

        return [True, 'The robot position has been reset successfully.']

def main():
    rospy.wait_for_service('/move_group/get_loggers')
    reset_robot_handler()
    rospy.spin()

if __name__ == '__main__':
    sys.exit(main())