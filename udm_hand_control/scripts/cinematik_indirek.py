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
    ## kinematic model and the robot's current pose states
    robot = moveit_commander.RobotCommander()

    ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    ## for getting, setting, and updating the robot's internal understanding of the
    ## surrounding world:
    scene = moveit_commander.PlanningSceneInterface()
    
    ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    ## to a planning group (group of poses).  
    ## This interface can be used to plan and execute motions:
    group_name = req.group
    ##  print "============ Robot Groups:"
    print robot.get_group_names()
    ##  print "============ Printing robot state"
    print robot.get_current_state()
    move_group = moveit_commander.MoveGroupCommander(group_name.data)
    print "Pose is: "
    print move_group.get_current_pose().pose
    print "Orientation is: "
    print req.orientation
    print "x is: "
    print req.posex
    print "y is: "
    print req.posey
    print "z is: "
    print req.posez

    ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
    ## trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)
    # We can get the pose values from the group and adjust some of the values:
    print "============ Generating plan.."
    pose_goal = geometry_msgs.msg.Pose()
    print pose_goal
    # plan a motion for this group
    pose_goal.orientation.w = req.orientation.data
    pose_goal.position.x = req.posex.data
    pose_goal.position.y = req.posey.data
    pose_goal.position.z = req.posez.data
    move_group.set_pose_target(pose_goal)

    # The go command can be called with pose values, poses, or without any
    # parameters if you have already set the pose or pose target for the group
    move_group.go(pose_goal, wait=True)
    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()
    # It is always good to clear your targets after planning with poses.
    move_group.clear_pose_targets()
    
    rep = udm_poseService()
    rep.message = "This action was planned at %s" %rospy.get_time()
    rep.res=True

    return rep


def server():
    ## a `rospy`_ node to declare a node using init_node() and then declare the udm_service service:
    rospy.init_node('cinematik_indirek', anonymous=True)
    s = rospy.Service('udm_poseService', udm_poseService, handle_req)
    ## rospy.spin() keeps code from exiting until the service is shutdown
    rospy.spin()


if __name__ == '__main__':
    server()
