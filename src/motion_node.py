#!/usr/bin/env python

# Motion Node
# Gabe Petersen - 12 April 2019
# Purpose: To create an empty shell that converts a trajectory 
# from QT visual interface and current vicon pose to cmd_velocity for duckiebot
# Should run on duckiebot itself since it will run roscore

import rospy
import math
from duckietown_msgs.msg import Twist2DStamped
from geometry_msgs.msg import Pose2D, Point

class VelocityConverter(object):
	def __init__(self):
		# default value
		self.carNum = 1
		if(not(rospy.has_param("car_number"))):
			rospy.loginfo("Error: car_number parameter cannot be found");

		rospy.get_param("car_number", self.carNum);
		sub_pose = "car" + str(self.carNum) + "/pose"
		sub_trajectory = "trajectory/car" + str(self.carNum)
		pub_velocity = "/car" + str(self.carNum) + "/cmd_vel"
		# setup subscriber and publisher
		self.pose_subscriber = rospy.Subscriber(sub_pose, Pose2D, self.pose_callback)
		self.trajectory_subscriber = rospy.Subscriber(sub_trajectory, Point, self.trajectory_callback)
		self.pub_car_cmd = rospy.Publisher(pub_velocity, Twist2DStamped, queue_size=1)

		# helper variables
		self.curr_pose = Pose2D()
	
	def pose_callback(self, pose_msg):
		curr_pose = pose_msg
	
	def trajectory_callback(self, trajectory_msg):
		x_final = trajectory_msg.x
		y_final = trajectory_msg.y
		cmd_vel = Twist2DStamped()

		# ----------------------------------------------------
		# ------------ Student Code Goes Here ----------------
		# ----------------------------------------------------

		# ----------------------------------------------------
		# ----------------------------------------------------

		self.pub_car_cmd.publish(cmd_vel)
	
if __name__ == '__main__':
    rospy.init_node('motion_node', anonymous=False)
    motion_node = VelocityConverter()
    rospy.spin()
