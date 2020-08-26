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

def handle_req(req):
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
    group_name = req.group
    ##  print "============ Robot Groups:"
    print robot.get_group_names()
    ##  print "============ Printing robot state"
    print robot.get_current_state()
    move_group = moveit_commander.MoveGroupCommander(group_name.data)
    print "Pose is: "
    print move_group.get_current_pose().pose
    
    # The group_thumb has only two valid joints, hence the if-else statement
    # if the group_thumb is selected, accept only two joint values
    # else 3 joint values
    if(group_name.data == "group_thumb"):
        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)
        # We can get the joint values from the group and adjust some of the values:
        joint_goal = move_group.get_current_joint_values()
        # print Joint values
        print move_group.get_current_joint_values()[0]
        print move_group.get_current_joint_values()[1]
        joint_goal[0] = req.joint_goal1.data
        joint_goal[1] = req.joint_goal2.data
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
    else: 
    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of joints).  
    ## This interface can be used to plan and execute motions:
        move_group = moveit_commander.MoveGroupCommander(group_name.data)
               ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

        # We can get the joint values from the group and adjust some of the values:
        joint_goal = move_group.get_current_joint_values()
        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        # print Joint values
        print move_group.get_current_joint_values()[0]
        print move_group.get_current_joint_values()[1]
        print move_group.get_current_joint_values()[2]
        joint_goal[0] = req.joint_goal1.data
        joint_goal[1] = req.joint_goal2.data
        joint_goal[2] = req.joint_goal3.data
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
    
    rep = udm_serviceResponse()
    rep.res = True
    rep.message = "This action was planned at %s" %rospy.get_time()

    return rep


def server():
    ## a `rospy`_ node to declare a node using init_node() and then declare the udm_service service:
    rospy.init_node('cinematik_direk', anonymous=True)
    s = rospy.Service('udm_service', udm_service, handle_req)
    ## rospy.spin() keeps code from exiting until the service is shutdown
    rospy.spin()


if __name__ == '__main__':
    server()
