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
	# initial constructor
	def __init__(self):
		# default value
		self.carNum = 1
		# intialize the graph
		self.graph_initialize()
		if(not(rospy.has_param("car_number"))):
			rospy.loginfo("Error: car_number parameter cannot be found")

		rospy.get_param("car_number", self.carNum)
		sub_pose = "car" + str(self.carNum) + "/pose"
		sub_trajectory = "trajectory/car" + str(self.carNum)
		pub_velocity = "/car" + str(self.carNum) + "/cmd_vel"
	
		# setup subscriber and publisher
		self.pose_subscriber = rospy.Subscriber(sub_pose, Pose2D, self.pose_callback)
		self.trajectory_subscriber = rospy.Subscriber(sub_trajectory, Point, self.trajectory_callback)
		self.pub_car_cmd = rospy.Publisher(pub_velocity, Twist2DStamped, queue_size=1)

	# helper variables
	curr_pose = Pose2D()
	graph_size = 0
	x_points = []
	y_points = []
	graph = []
	
	def pose_callback(self, pose_msg):
		curr_pose = pose_msg
	
	def trajectory_callback(self, trajectory_msg):
		# get message and declare some helper variables
		desired_point = Point()
		desired_point = trajectory_msg;
		# start velocity commands
		self.localization(desired_point);
	
	# function that intializes the graph of duckietown
	# utilizes graph_add_vertex and graph_edge_vertex
	def graph_initialize(self):
		###########################
		# DECLARE GRAPH SIZE HERE #	
		self.graph_size = 5
		###########################
		
		self.x_points = []
		self.y_points = []
		self.graph = []
		# Make the graph, x_points, and y_points functional arrays
		for i in range(0,self.graph_size):
			self.graph.append([0 for i in range(self.graph_size)])
			self.x_points.append(0)
			self.y_points.append(0)
		
		for i in range(0,self.graph_size):
			for j in range(0,self.graph_size):
				self.graph[i][j] = 0
	
		# intialize vertices
		self.graph_add_vertex(0, 0.0, 0.0)
		self.graph_add_vertex(1, 4.6, 3.4)
		self.graph_add_vertex(2, 0.4, 2.3)
		self.graph_add_vertex(3, 3.2, 6.7)
		self.graph_add_vertex(4, 1.2, 3.4)
		# intialize edge graph
		# specify indicies for graph_add_edge
		self.graph_add_edge(0, 1)
		self.graph_add_edge(0, 4)
		self.graph_add_edge(1, 3)
		self.graph_add_edge(1, 2)
		self.graph_add_edge(2, 3)
		self.graph_add_edge(3, 4)

	# function to calculate the linear distance between two points
	def lin_dist(self, xi, xf, yi, yf):
		return pow((pow((xf - xi),2) + pow((yf - yi),2)), 0.5)
	
	# function to add a vertex into the graph of duckietown
	def graph_add_vertex(self, vertex_index, x_val, y_val):
		# if the inputted vertex_index is not valid, print error
		if vertex_index >= self.graph_size:
			rospy.loginfo("Error: vertex index is not valid - exceeds graph size")
			return
		self.x_points[vertex_index] = x_val
		self.y_points[vertex_index] = y_val

	# function to add an edge to the graph of duckietown
	def graph_add_edge(self, vi1, vi2):
		self.graph[vi1][vi2] = self.graph[vi2][vi1] = self.lin_dist(self.x_points[vi1], self.x_points[vi2], self.y_points[vi1], self.y_points[vi2])
	
	# function to find a vertex in the duckietown graph corresponding to a particular point
	def graph_find_vertex(self, x, y):
		# closeness threshold is how close the input needs to be to the actual point
		closeness_thresh = 0.1
		for i in range(0, self.graph_size):
			if ((abs(self.x_points[i] - x) < closeness_thresh ) and (abs(self.y_points[i] - y) < closeness_thresh)):
				return i
		# if you cannot find the vertex in the graph return -1
		return -1

	# function to apply dixtra's shortest path algorithm
	# from website: https://www.geeksforgeeks.org/printing-paths-dijkstras-shortest-path-algorithm/
	def graph_dixtra(self, source_index, parent_path, dist):
		sepSet = []
		
		# initialize data structures
		for i in range(0, self.graph_size):
			dist[i] = float("inf")
			sepSet.append(False)
			parent_path[source_index] = -1;
		# set the source distance to be zero - processed first
		dist[source_index] = 0
		for i in range(0, self.graph_size):
			# find the vertex with min_distance from source from available vertices
			# set this index as u
			min_dist = float("inf")
			for v in range(0, self.graph_size):
				if sepSet[v] == False and dist[v] <= min_dist:
					min_dist = dist[v]
					u = v
			# set value that we visited u = true
			sepSet[u] = True
			# update distance values adjacent to this vertex
			for v in range(0, self.graph_size):
				# if v is not visited already, there is not already an edge from u to v
			   # and the distance from src to u to v is smaller than current distance from
			   # src to vertex v, then update the value
			   # and...store vertex u in parent array at index v
				if(not(sepSet[v]) and self.graph[u][v] and (dist[v] > dist[u] + self.graph[u][v])):
					parent_path[v] = u
					dist[v] = dist[u] + self.graph[u][v]
	
	# Function to print shortest 
	# path from source to j 
	# using parent array 
	# from website: https://www.geeksforgeeks.org/printing-paths-dijkstras-shortest-path-algorithm/
	def printPath(self, parent, j):
		if parent[j] == -1:
			return
		self.printPath(parent, parent[j])
		## THIS IS BUGGY BC PRINT LIKES TO MAKE A NEWLINE EVERYTIME ITS CALLED
		print(str(j))

	# Algorithm from https://www.geeksforgeeks.org/printing-paths-dijkstras-shortest-path-algorithm/  
	# A utility function to print  
   # the constructed distance array 
	def printSolution(self, src, dist, n, parent):
		print("Vertex\tDistance\tPath")
		for i in range(0,n):
			print("\n" + str(src) + " -> " + str(i) + "\t\t" + str(dist[i]) + "\t\t" + str(src))
			self.printPath(parent, i)
	
	# localization gets the current point and desired point
	# Uses dixtra's algorithm to find the shortest path between them
	# Uses a recursive helper to segment the velocities
	def localization(self, desired_point):
		# find the vertex near the source
		src = self.graph_find_vertex(self.curr_pose.x, self.curr_pose.y)
		# find the vertex near the end point
		fin = self.graph_find_vertex(desired_point.x, desired_point.y)
		# make parent and distances array accessible
		parent = []
		distances = []
		for i in range(0, self.graph_size):
			parent.append(0)
			distances.append(0)
		
		# calculate dixtra's algorithm
		self.graph_dixtra(src, parent, distances)
		#
		# PRINT FOR DEBUGGING
		#
		self.printSolution(src, distances, self.graph_size, parent)
		print("\nGRAPH OF DISTANCES\n")
		for i in range(0, self.graph_size):
			for j in range(0, self.graph_size):
				# THIS IS BUGGY BC PRINT LIKES TO PRINT NEWLINE
				print(str(self.graph[i][j]) + "\t")
		print("\nx_points, y_points for duckietown\n")
		for i in range(0, self.graph_size):
			print("( " + str(self.x_points[i]) + ", " + str(self.y_points[i]) + " )")
		print("\nTo go from location: index - " + str(src) + ": (" + str(self.x_points[src]) + ", " + str(self.y_points[src]) + ") to index - " + str(fin) + ": (" + str(self.x_points[fin]) + ", " + str(self.y_points[fin]) + ") .....")
		#
		# PRINT FOR DEBUGGING
		#
		# call recursive algorithm to control velocities
		self.localizationHelper(parent, fin)
	
		
	# localizationHelper is a recursive alrogithm to segment velocity controls
   # Takes in an array of parents to the shortest path and controls the duckiebot to each point
   # Uses a recursive helper to segment the velocities
	def localizationHelper(self, parent, u):
		# base case - path is at source node (current_pose)
		if parent[u] == -1: 
			return
		# ---------- NEED TO INITILIAZE THESE PARAMETERS ----------
		# control what area around the desired point where the duckiebot should stop
		diff_threshold = 0.1
		# how powerful the linear velocity should be based on a specific distance
		v_gain = 1.0
		# how powerful the angular velocity should be based on a specific distance 
		omega_gain = 1.0
		# parameters based on the distance the duckiebot is told to actually turn
		# These need to actually be measured based on the duckietown turn/intersection tiles
		x_turn_threshold = 0.1
		y_turn_threshold = 0.1
		# Declare publisher variable
		velocity_command = Twist2DStamped()
		velocity_command.v = 0
		velocity_command.omega = 0

	   # CURRENT desired point is vertex u
		desired_point = Point()
		desired_point.x = self.x_points[u]
		desired_point.y = self.y_points[u]
	
		# if difference between desired_point and current point is big enough
		if (pow( (pow(desired_point.x,2) + pow(desired_point.y,2)),0.5) - pow((pow(self.curr_pose.x,2) + pow(self.curr_pose.y,2)),0.5) > diff_threshold):
			# calculate current velocity is based off how far away the object is
			velocity_command.v = v_gain * self.lin_dist(self.curr_pose.x, desired_point.x, self.curr_pose.y, desired_point.y)
			# test if the duckiebot needs to turn
			# determine if it needs to make a left or right turn based on where the actual desired point is relative to current_pose
			# x_turn_threshold and y_turn_threshold control this a lot
			if(((desired_point.x - self.curr_pose.x) >= x_turn_threshold) and ((desired_point.y - self.curr_pose.y) >= y_turn_threshold)):
				# if current_pose = horizontal		
				velocity_command.omega = omega_gain * (self.curr_pose.theta - (math.pi / 2))
			elif(((desired_point.x - self.curr_pose.x) <= (-1 * x_turn_threshold)) and ((desired_point.y - self.curr_pose.y) <= (-1 * y_turn_threshold))):
				velocity_command.omega = omega_gain * (self.curr_pose.theta - (math.pi / 2));
			elif(((desired_point.x - self.curr_pose.x) >= x_turn_threshold) and ((desired_point.y - self.curr_pose.y) <= (-1 * y_turn_threshold))):
				velocity_command.omega = omega_gain * (self.curr_pose.theta + (math.pi / 2));
			elif( ((desired_point.x - self.curr_pose.x) <= (-1 * x_turn_threshold)) and ((desired_point.y - self.curr_pose.y) >= y_turn_threshold)):
				velocity_command.omega = omega_gain * (self.curr_pose.theta + (math.pi / 2));
			else:
				velocity_command.omega;

			# if the car is horizontal, negate the omega speed
			if( (abs(self.curr_pose.theta) < (math.pi / 6)) or (abs(self.curr_pose.theta) > ((5 * math.pi) / 6)) ):
				velocity_command.omega = -1 * velocity_command.omega;
		# if the difference between the desired point and current point is small enough
		else:
			# set the velocity equal to 0
			velocity_command.v = 0;

		next_u = parent[u]
		# recursive call to complete next steps of mission
		self.localizationHelper(parent, next_u);

		print("publish velocity: v = " + str(velocity_command.v) + ", omega = " + str(velocity_command.omega) + "\n")
		# publish the velocities
		self.pub_car_cmd.publish(velocity_command);

		# latch time until duckiebot completes its mission
		while((desired_point.x - self.curr_pose.x > 0.1) and (desired_point.y - self.curr_pose.y > 0.1)):
			count = 1
		
	
if __name__ == '__main__':
    rospy.init_node('motion_node', anonymous=False)
    motion_node = VelocityConverter()
    rospy.spin()
