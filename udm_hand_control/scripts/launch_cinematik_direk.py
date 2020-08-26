#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from udm_hand_control.srv import *


class kinematics_fingers:
    def __init__(self, group_name, joint1, joint2, joint3):
        # initialise variables
        self.group_name = group_name
        self.joint1 = joint1
        self.joint2 = joint2
        self.joint3 = joint3

        ## a `rospy`_ node to declare a node using init_node() and then declare the udm_service service:
        rospy.init_node('cinematik_direk', anonymous=True)

        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv) 

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()
        
        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  
        ## This interface can be used to plan and execute motions:
        group_name = group_name
        ##  print "============ Robot Groups:"
        print robot.get_group_names()
        ##  print "============ Printing robot state"
        print robot.get_current_state()
        move_group = moveit_commander.MoveGroupCommander(group_name)
        print "Pose is: "
        print move_group.get_current_pose().pose
        
        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                moveit_msgs.msg.DisplayTrajectory,
                                                queue_size=20)
        # We can get the joint values from the group and adjust some of the values:
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = joint1
        joint_goal[1] = joint_goal2
        joint_goal[2] = joint_goal3

        # print Joint values
        print move_group.get_current_joint_values()[0]
        print move_group.get_current_joint_values()[1]
        print move_group.get_current_joint_values()[2]

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()
        # For testing:
        current_joints = move_group.get_current_joint_values()
        # print Joint values
        print move_group.get_current_joint_values()[0]
        print move_group.get_current_joint_values()[1]
        print move_group.get_current_joint_values()[2]


class kinematics_thumb:
    def __init__(self, group_name, joint1, joint2):
        # initialise variables
        self.group_name = group_name
        self.joint1 = joint1
        self.joint2 = joint2

        ## a `rospy`_ node to declare a node using init_node() and then declare the udm_service service:
        rospy.init_node('cinematik_direk', anonymous=True)

        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv) 

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()
        
        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  
        ## This interface can be used to plan and execute motions:
        group_name = group_name
        ##  print "============ Robot Groups:"
        print robot.get_group_names()
        ##  print "============ Printing robot state"
        print robot.get_current_state()
        move_group = moveit_commander.MoveGroupCommander(group_name)
        print "Pose is: "
        print move_group.get_current_pose().pose
        
        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                moveit_msgs.msg.DisplayTrajectory,
                                                queue_size=20)
        # We can get the joint values from the group and adjust some of the values:
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = joint1
        joint_goal[1] = joint_goal2

        # print Joint values
        print move_group.get_current_joint_values()[0]
        print move_group.get_current_joint_values()[1]

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()
        # For testing:
        current_joints = move_group.get_current_joint_values()
        # print Joint values
        print move_group.get_current_joint_values()[0]
        print move_group.get_current_joint_values()[1]
        print move_group.get_current_joint_values()[2]


if __name__ == '__main__':
    kinematics_fingers('group_index', 8.0, -7.0, -7.0)
    kinematics_fingers('group_middle', 0.0, 0.0, 0.0)
    kinematics_fingers('group_ring', 0.0, 0.0, 0.0)
    kinematics_fingers('group_pinky', 0.0, 0.0, 0.0)
    kinematics_thumb('group_thumb', 0.0, 0.0)
