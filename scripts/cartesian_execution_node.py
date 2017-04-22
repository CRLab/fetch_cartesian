#!/usr/bin/env python

import sys
import importlib
import copy

import rospy
import actionlib
import moveit_commander
import moveit_msgs.msg
import control_msgs.msg
import copy



from reachability_analyzer.grasp_reachability_analyzer import GraspReachabilityAnalyzer

from moveit_python import (MoveGroupInterface,
                           PlanningSceneInterface,
                           PickPlaceInterface)
from moveit_python.geometry import rotate_pose_msg_by_euler_angles
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
import trajectory_msgs.msg

class CartesianExecutionNode: 
    def __init__(self, node_name='cartesian_execution_node'): 
         # Initialize the move_group API 
         moveit_commander.roscpp_initialize(sys.argv) 

         # Initialize the ROS node 
         rospy.init_node(node_name) 

         self.cartesian = rospy.get_param('~cartesian', True) 

         # Connect to the self.arm move group 
         self.arm = moveit_commander.MoveGroupCommander(rospy.get_param('move_group_name')) 

         # Allow replanning to increase the odds of a solution 
         self.arm.allow_replanning(True) 

         # set the frame of reference for the self.arm
         self.arm.set_pose_reference_frame('base_link') 

         # Allow some leeway in position(meters) and orientation (radians) 
         self.arm.set_goal_position_tolerance(0.01) 
         self.arm.set_goal_orientation_tolerance(0.1) 

         # Get the name of the end-effector link 
         self.end_effector_link = self.arm.get_end_effector_link() 

         #start location
         self.arm.set_named_target('home') 

         # go to the start location
         self.arm.go()

    def set_waypoint(self, start_pose): 
         waypoints = []
         # set the first waypoint to be the starting pose 
         if self.cartesian: 
             # append the pose to the waypoints list 
             waypoints.append(start_pose) 

         wpose = copy.deepcopy(start_pose) 

         # Set the next waypoint back 0.2 meters and right 0.2 meters 
         wpose.position.x -= 0.1 
         # wpose.position.y -= 0.1 

         if self.cartesian: 
             waypoints.append(copy.deepcopy(wpose)) 
         else: 
             self.arm.set_pose_target(wpose) 
             self.arm.go() 
             rospy.sleep(1)

         # set next waypoint 
         # wpose.position.x += 0.05 
         wpose.position.y += 0.1 
         # wpose.position.z -= 0.15 

         if self.cartesian: 
             waypoints.append(copy.deepcopy(wpose)) 
         else: 
             self.arm.set_pose_target(wpose) 
             self.arm.go() 
             rospy.sleep(1) 

         if self.cartesian: 
             waypoints.append(copy.deepcopy(start_pose)) 
         else: 
             self.arm.set_pose_target(start_pose) 
             self.arm.go() 
             rospy.sleep(1) 

         if self.cartesian: 
             fraction = 0.0 
             maxtries = 100 
             attempts = 0 

             # Set the internal state to the current state 
             self.arm.set_start_state_to_current_state() 

             # Plan the Cartesian path connecting the waypoints 
             while fraction < 1.0 and attempts < maxtries: 
                 (plan, fraction) = self.arm.compute_cartesian_path ( 
                                         waypoints,   # waypoint poses 
                                         0.01,        # eef_step 
                                         0.0,         # jump_threshold 
                                         True)        # avoid_collisions 

                 # Increment the number of attempts 
                 attempts += 1 

                 # Print out a progress message 
                 if attempts % 10 == 0: 
                     rospy.loginfo("Still trying after " + str(attempts) + " attempts...") 

             # If we have a complete plan, execute the trajectory 
             if fraction == 1.0: 
                 rospy.loginfo("Path computed successfully. Moving the self.arm.") 

                 self.arm.execute(plan) 

                 rospy.loginfo("Path execution complete.") 
             else: 
                 rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.") 

         # Move normally back to the 'resting' position 
         import IPython
         IPython.embed()
         self.arm.set_named_target('home') 
         self.arm.go() 
         rospy.sleep(1) 

         # Shut down MoveIt cleanly 
         moveit_commander.roscpp_shutdown() 

         # Exit MoveIt 
         moveit_commander.os._exit(0) 

if __name__ == "__main__": 
     try: 
         ce = CartesianExecutionNode()
         #get current pose to start
         #then we can set different
         start_pose = ce.arm.get_current_pose(ce.end_effector_link).pose
         ce.set_waypoint(start_pose)
     except rospy.ROSInterruptException: 
         pass 