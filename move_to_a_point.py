#!/usr/bin/env python

import numpy as np
from numpy import linalg as LA

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose
from std_msgs.msg import String


class Move2Point:

    def __init__(self):
        self.vel_d = np.zeros([3, 1])
        self.omega_d = np.zeros([3, 1])
        self.position = np.zeros([3, 1])
        self.orientation = np.zeros([4, 1])
        self.velocity = np.zeros([3, 1])

        # subscriber
        self.pose_sub = rospy.Subscriber('/lwr/ee_pose', Pose, self.update_pose)
        self.pose_twist = rospy.Subscriber('/lwr/ee_vel', Pose, self.update_twist)

        # publisher
        self.vel_pub = rospy.Publisher('/lwr/joint_controllers/passive_ds_command_vel',
                                       Twist, queue_size=10)
        self.ori_pub = rospy.Publisher('/lwr/joint_controllers/passive_ds_command_orient',
                                       Quaternion, queue_size=10)


    def compute_command(self):
        """ """
        distance = LA.norm(-self.position)

        self.vel_d = 
        self.omega_d =

        return

    def publish_data(self):
        """call back function """

        self.vel_pub.publish()

        self.ori_pub.publish()


    def update_pose(self, data):
        self.position = np.array([data.position.x,
                                         data.position.y,
                                         data.position.z])
        self.orientation = np.array([data.orientation.w,
                                            data.orientation.x,
                                            data.orientation.y,
                                            data.orientation.z])

    def update_twist(self, data):
        self.velocity = np.array([data.linear.x,
                                  data.linear.y,
                                  data.linear.z])

    def move2goal(self):
        """ """
        self.goal_pose = Pose()

        # Get the input from the user.
        # self.goal_pose.x = input("Set your x goal: ")
        # self.goal_pose.y = input("Set your y goal: ")
        # self.goal_pose.y = input("Set your y goal: ")

        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        distance_tolerance = input("Set your tolerance: ")


        # Running
        self.compute_command()
        self.publish_data()

        # Stopping our robot after the movement is over.

        # If we press control + C, the node will stop.
        rospy.spin()