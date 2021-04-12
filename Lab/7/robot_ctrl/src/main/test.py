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
File Name: test.py
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
from std_srvs.srv import Trigger
from moveit_msgs.msg import DisplayTrajectory

# Package creates links between the target and the object
import copy

# Numpy (Array computing Lib.)
import numpy as np

def joint_trajectory(group, joint):
    """
    Description:
        A simple demonstration of a joint control (trajectory planning from specified joint positions).

        More information about MoveIt at:
        https://moveit.ros.org/
    Args:
        (1) group [MoveGroup Class]: Client class for the MoveGroup action.
        (2) joint [Float Array]: Array of robot joints.

    Returns:
        (1) param{1} [INT]: The representation of a motion plan (time, start state, trajectory, etc).
    """

    # Set the parameters of the Robot (Joint Control)
    group.set_joint_value_target(joint)

    # Get the planned trajectory
    plan = group.plan(joint)

    return plan

def cartesian_trajectory_1(group, cartesian):
    """
    Description:
        A simple demonstration of a cartesian control (trajectory planning from specified cartesian positions).

        More information about MoveIt at:
        https://moveit.ros.org/
    Args:
        (1) group [MoveGroup Class]: Client class for the MoveGroup action.
        (2) cartesian [Float Array]: Array of robot cartesian positions (x, y, z, Quaternion(w..z)).

    Returns:
        (1) param{1} [INT]: The representation of a motion plan (time, start state, trajectory, etc).
    """

    # Set the parameters of the Robot (TCP/Cartesian Control)
    cartesian_target = geometry_msgs.msg.Pose()
    # Position
    cartesian_target.position.x = cartesian[0]
    cartesian_target.position.y = cartesian[1]
    cartesian_target.position.z = cartesian[2]
    # Orientation
    cartesian_target.orientation.w = cartesian[3]
    cartesian_target.orientation.x = cartesian[4]
    cartesian_target.orientation.y = cartesian[5]
    cartesian_target.orientation.z = cartesian[6]
    group.set_pose_target(cartesian_target)

    # Get the planned trajectory
    plan = group.plan()

    return plan

def cartesian_trajectory_2(group, cartesian, offset, vel_f, acc_f, step, allow_collision):
    """
    Description:
       A simple example of Cartesian control of several points (trajectory planning from specified initial Cartesian positions). 
       Other parameters are calculated from the function (example of a triangle path).

        More information about MoveIt at:
        https://moveit.ros.org/
    Args:
        (1) group [MoveGroup Class]: Client class for the MoveGroup action.
        (2) cartesian [Float Array]: Array of robot cartesian positions (x, y, z, Quaternion(w..z)).
        (3) offset [Float Array]: Offset positions (x, y, z).
        (4 - 5) vel_f, acc_f [Float]: Velocity/Acceleration scaling factor.
        (6) step [INT]: Number of steps (Cartesian calculation).
        (7) allow_collision [Bool]: Allow collision object or not.

    Returns:
        (1) param{1} [INT]: The representation of a motion plan (time, start state, trajectory, etc).
    """

    waypoints = []
    w_pose_target  = geometry_msgs.msg.Pose()

    # Initialize the robot's orientation on the entire trajectory
    w_pose_target.orientation.w = cartesian[3]
    w_pose_target.orientation.x = cartesian[4]
    w_pose_target.orientation.y = cartesian[5]
    w_pose_target.orientation.z = cartesian[6]

    # Waipoint (1)
    w_pose_target.position.x = cartesian[0] - offset[0]
    w_pose_target.position.y = cartesian[1] + offset[1]
    w_pose_target.position.z = cartesian[2] - offset[2]
    waypoints.append(copy.deepcopy(w_pose_target))
    # Waipoint (2)
    w_pose_target.position.x = cartesian[0] + offset[0]
    w_pose_target.position.y = cartesian[1] - offset[1]
    w_pose_target.position.z = cartesian[2] - offset[2]
    waypoints.append(copy.deepcopy(w_pose_target))
    # Waipoint (3)
    w_pose_target.position.x = cartesian[0]
    w_pose_target.position.y = cartesian[1]
    w_pose_target.position.z = cartesian[2]
    waypoints.append(copy.deepcopy(w_pose_target))

    # Calculate the Cartesian path from waipoints 1 - 3
    (plan, fraction) = group.compute_cartesian_path(waypoints, step, 0.0, avoid_collisions = allow_collision, path_constraints = None)

    if 1-fraction < 0.01:
        rospy.loginfo('Path computed successfully.')
    else:
        rospy.loginfo('Path planning failed')

    rospy.loginfo('Fraction: %f' % fraction)

    # Get the planned trajectory (Re-Time TCP/Cartesian Control -> multiple points)
    plan = group.retime_trajectory(group.get_current_state(), plan, vel_f, acc_f)

    return plan

def main():
    # Robot Node initialization
    rospy.init_node('robot_init', anonymous=True)   

    # Visible -> on/off in the RVIZ environment
    visible_object = False
    rospy.set_param('object_visible', visible_object)

    # Moveit Initialization
    group = moveit_commander.MoveGroupCommander('manipulator')
    scene = moveit_commander.PlanningSceneInterface()
    robot = moveit_commander.RobotCommander()
    
    # Reset robot positions (go to home position)
    rospy.wait_for_service('/reset_robot')
    reset_robot  = rospy.ServiceProxy('/reset_robot', Trigger)
    reset_robot.call()

    rospy.sleep(0.5)
    
    # Initialize the current position of the robot
    w_pose_initial = rospy.wait_for_message('/current_tcp_pose', geometry_msgs.msg.PoseStamped, timeout=None)
    position       = [w_pose_initial.pose.position.x, w_pose_initial.pose.position.y, w_pose_initial.pose.position.z]
    orientation    = [w_pose_initial.pose.orientation.w, w_pose_initial.pose.orientation.x, w_pose_initial.pose.orientation.y, w_pose_initial.pose.orientation.z]

    # Set the parameters of the Object (display -> environment)
    rospy.set_param('object_pos', position)

    # Setting parameters for planning
    vel_scaling_f = 1.0
    acc_scaling_f = 1.0
    group.set_max_velocity_scaling_factor(vel_scaling_f)
    group.set_max_acceleration_scaling_factor(acc_scaling_f)
    # Planner -> OMPL (Default) or BiTRRTkConfigDefault
    group.set_planner_id('OMPL')

    # Joint, Cartesian_1, Cartesian_2 or None
    mode = 'Cartesian_1'

    if mode == 'Joint':
        # Generate Joint Trajectory
        plan = joint_trajectory(group, [1.57, -1.57, 1.57, -1.57, -1.57, 0.0])

    elif mode == 'Cartesian_1':
        # Generate Cartesian Trajectory (1)
        plan = cartesian_trajectory_1(group, 
                [position[0], position[1] - 0.1, position[2], 
                orientation[0], orientation[1], orientation[2], orientation[3]]
        )

    elif mode == 'Cartesian_2':
        # Generate Cartesian Trajectory (2)
        plan = cartesian_trajectory_2(group, 
                [position[0], position[1], position[2], 
                orientation[0], orientation[1], orientation[2], orientation[3]],
                [0.05, 0.25, 0.30],
                vel_scaling_f, acc_scaling_f, 0.01, True
        )

    if mode != 'None':
        rospy.loginfo('Intermediate points on the robot trajectory: %f' % len(plan.joint_trajectory.points))

        # Show trajectory
        display_trajectory = DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)

        # Execute the trajectory
        group.execute(plan, wait=True)
        rospy.sleep(0.5)

    # Reset
    group.stop()
    group.clear_path_constraints()
    group.clear_pose_targets()

if __name__ == '__main__':
    sys.exit(main())