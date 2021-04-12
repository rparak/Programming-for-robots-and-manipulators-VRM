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
File Name: set_object.py
## =========================================================================== ## 
"""

# System (Default Lib.)
import sys

# Python client library for ROS
import rospy

# Package offers wrappers for the functionality provided in MoveIt
import moveit_commander

# Data types (messages, services)
import geometry_msgs

def main():
    # Node initialization
    rospy.init_node('set_object_init', anonymous=True)

    group = moveit_commander.MoveGroupCommander('manipulator')
    scene = moveit_commander.PlanningSceneInterface()
    robot = moveit_commander.RobotCommander()

    try:
        while not rospy.is_shutdown():
            visibility = rospy.get_param('/object_visible')

            if visibility == True:
                # Get the parameters from the main script (see test.py)
                position = rospy.get_param('/object_pos')

                # Set the parameters of the Object (RVIZ)
                obj = geometry_msgs.msg.PoseStamped()
                obj.header.frame_id = robot.get_planning_frame()
                obj.pose.position.x = position[0]
                obj.pose.position.y = position[1]
                obj.pose.position.z = position[2] - 0.3

                # Display the object in the environment
                scene.add_sphere('obj_1', obj, 0.05)
            else:
                scene.remove_world_object()

            # rospy rate sleep -> 10 Hz
            rospy.Rate(10).sleep()
    except:
        print('Unexpected error:', sys.exc_info()[0])
    finally:
        print('Bye.')

if __name__ == '__main__':
    sys.exit(main())