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

        # init the node
        rospy.init_node('move2point', anonymous=True)

        self.vel_d = np.zeros([3, 1])
        self.omega_d = np.zeros([3, 1])
        self.position = np.zeros([3, 1])
        self.orientation = np.zeros([4, 1])
        self.velocity = np.zeros([3, 1])
        self.position_goal = np.zeros([3, 1])

        # subscriber
        self.pose_sub = rospy.Subscriber('/lwr/ee_pose', Pose, self.update_pose)
        self.pose_twist = rospy.Subscriber('/lwr/ee_vel', Twist, self.update_twist)

        # publisher
        self.vel_pub = rospy.Publisher('/lwr/joint_controllers/passive_ds_command_vel',
                                       Twist, queue_size=1)
        self.ori_pub = rospy.Publisher('/lwr/joint_controllers/passive_ds_command_orient',
                                       Quaternion, queue_size=1)
        self.rate = rospy.Rate(200)

    def compute_command(self):
        """ """
        distance_m = self.position_goal-self.position

        # print(self.position_goal)
        # print(self.position)
        # # print('distance')
        # print(distance_m)
        b_m = np.identity(3)
        l_m = np.identity(3)*10
        l_m[2, 2] = 100

        self.vel_d = 3.0*distance_m
        if(LA.norm(self.vel_d)>0.3):
            self.vel_d = self.vel_d*0.3/LA.norm(self.vel_d)
        print(self.vel_d)
        self.omega_d = np.zeros([3, 1])

        # Desired quaternion to have the end effector looking down
        self.orientation[2, 0] = 1.0

    def publish_data(self):
        """call back function """
        msg_desired_twist = Twist()

        # msg_desired_twist.linear = self.vel_d
        # print(self.position)
        # print(self.vel_d[0, 0])
        msg_desired_twist.linear.x = self.vel_d[0, 0]
        msg_desired_twist.linear.y = self.vel_d[1, 0]
        msg_desired_twist.linear.z = self.vel_d[2, 0]
        msg_desired_twist.angular.x = self.omega_d[0, 0]
        msg_desired_twist.angular.y = self.omega_d[1, 0]
        msg_desired_twist.angular.z = self.omega_d[2, 0]

        self.vel_pub.publish(msg_desired_twist)

        msg_desired_orientation = Quaternion()

        # print(float(self.orientation[0]))
        msg_desired_orientation.w = 0
        msg_desired_orientation.x = 0
        msg_desired_orientation.y = 1
        msg_desired_orientation.z = 0

        self.ori_pub.publish(msg_desired_orientation)

    def update_pose(self, data):
        self.position = np.array([[data.position.x],
                                  [data.position.y],
                                  [data.position.z]])
        # print(self.position)
        self.orientation = np.array([[data.orientation.w],
                                     [data.orientation.x],
                                     [data.orientation.y],
                                     [data.orientation.z]])
        # print(self.orientation)

    def update_twist(self, data):
        self.velocity = np.array([[data.linear.x],
                                  [data.linear.y],
                                  [data.linear.z]])

    def move2goal(self):
        """ """
        # Get the input from the user.
        self.position_goal[0] = input("Set your x goal: ")
        self.position_goal[1] = input("Set your y goal: ")
        self.position_goal[2] = input("Set your z goal: ")
        # self.position_goal = np.concatenate([np.array(i) for i in self.position_goal])

        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        distance_tolerance = input("Set your tolerance: ")

        # Running

        while not rospy.is_shutdown():

            # print(self.position_goal - self.position)
            print(LA.norm(self.position_goal - self.position))
            self.compute_command()
            self.publish_data()

            self.rate.sleep()
        rospy.spin()

        # Stopping our robot after the movement is over.
        # self.vel_d = np.zeros([3, 1])
        # self.omega_d = np.zeros([3, 1])
        # self.orientation = np.zeros([4, 1])
        # self.publish_data()

        # If we press control + C, the node will stop.


if __name__ == '__main__':
    try:
        x = Move2Point()
        x.move2goal()
    except rospy.ROSInterruptException:
        pass
