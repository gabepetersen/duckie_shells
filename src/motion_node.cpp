// Gabe Petersen
// motion_node.cpp - 12 Apr 2019
// Purpose: To create an empty shell that converts a trajectory 
// from QT visual interface and current vicon pose to cmd_velocity for duckiebot
// Should run on duckiebot itself since it will run roscore

#include "ros/ros.h"
#include <math.h>
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Point.h"																		
#include "geometry_msgs/Twist.h"		
#include "duckietown_msgs/Twist2DStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <vector>
#include <queue>
#include <limits>

class VelocityConverter {
	public:
		VelocityConverter() {
			// private nodehandler
			ros::NodeHandle nhp("~");
			// Check if parameters exist
			if(!nhp.hasParam("car_number")) {
				ROS_INFO("Error: Parameter for Car Number Cannot be found");
			}
			// default car number
			carNum = 1;
			// Here is the sepcification of the graph size
			graph_size = 5;
			// get the car number and do some string operations to get correct subscription/publisher topics
			graph_initialize();
			nhp.getParam("car_number", carNum);
			std::stringstream sub_pose, sub_trajectory, pub_vel;
			sub_pose << "/car" << carNum << "/pose";
			sub_trajectory << "/trajectory/car" << carNum;
			pub_vel << "/car" << carNum << "/cmd_vel";

			// subscribe to the pose of the duckiebot number specified by the parameters
			pose_subscriber = nh.subscribe(sub_pose.str(), 1, &VelocityConverter::pose_callback, this);
			trajectory_subscriber = nh.subscribe(sub_trajectory.str(), 1, &VelocityConverter::trajectory_callback, this);
			velocity_publisher = nh.advertise<duckietown_msgs::Twist2DStamped>(pub_vel.str(), 1);
		}

		void pose_callback(const geometry_msgs::Pose2D::ConstPtr& msg);
		void trajectory_callback(const geometry_msgs::Point::ConstPtr& msg);
		void graph_initialize();	/// Have to edit for point initialization
		void graph_add_edge(int vi1, int vi2);
		void graph_add_vertex(int vertex_index, float x_val, float y_val);
		int  graph_vertex_index(float x, float y);
		void graph_dixtra(int source_index, int parent_path[], float dist[]);
		int graph_find_vertex(float x, float y);
		void localization(geometry_msgs::Point desired_point);
		void localizationHelper(int parent[], int u);
		float lin_dist(float xi, float xf, float yi, float yf);
			
	private:
		// ros variables
		ros::NodeHandle nh;
		ros::Subscriber pose_subscriber;
		ros::Subscriber trajectory_subscriber;
		ros::Publisher velocity_publisher;

		// helper variables
		geometry_msgs::Pose2D current_pose;
		int carNum;
		int graph_size;
		/// actual points on duckietown
		float x_points[100];
		float y_points[100];
		float graph[100][100];
};

int main(int argc, char** argv) {
	// initialize the node
	ros::init(argc, argv, "motion_node");
	// instantiate the class
	VelocityConverter vc;

	// declare sample rate
	ros::Rate r(100);
	while(ros::ok()) {
		r.sleep();
		ros::spinOnce();
	}	
	return 0;
}

// sets value of current pose
void VelocityConverter::pose_callback(const geometry_msgs::Pose2D::ConstPtr& msg) {
	// possibly error check
	current_pose = *msg;
}

/* Gets value of desired trajectory from final destination point
 * Coupled with current_pose, it computes the cmd_vel needed for
 * the duckiebot
 */
void VelocityConverter::trajectory_callback(const geometry_msgs::Point::ConstPtr& msg) {
	// get message and declare some helper variables
	geometry_msgs::Point desired_point = *msg;
	// start velocity commands
	localization(desired_point);
}
/* GraphInitialize is a manually-entered function designed to intialize a graph
 * with an adjacency matrix based on the duckietown layout
 * Beginning/End points at intersections should be specified in the graph
 * Beginning/End points of curves should be specified in the graph
 * Edges can be added through the add_edge_function - specifiying the vertices
 */
void VelocityConverter::graph_initialize() {
	/// clear graph first
	for(int i = 0; i < graph_size; i++) {
		for(int j = 0; j < graph_size; j++) {
			graph[i][j] = 0;
		}
	}
	/// intialize verticies
	graph_add_vertex(0, 0.0, 0.0);
	graph_add_vertex(1, 4.6, 3.4);
	graph_add_vertex(2, 0.4, 2.3);
	graph_add_vertex(3, 3.2, 6.7);
	graph_add_vertex(4, 1.2, 3.4);
	/// intialize edge graph
	/// specify indicies for graph_add_edge
	graph_add_edge(0, 1);
	graph_add_edge(0, 4);
	graph_add_edge(1, 3);
	graph_add_edge(1, 2);
	graph_add_edge(2, 3);
	graph_add_edge(3, 4);
}
/* Takes x_points and y_points in class instantiation
 * along with the two vertex indicies and then computes the length between them
 * It stores this length in the adjacency matrix
 */
void VelocityConverter::graph_add_edge(int vi1, int vi2) {
	graph[vi1][vi2] = graph[vi2][vi1] = lin_dist(x_points[vi1], x_points[vi2], y_points[vi1], y_points[vi2]);
}
/* Takes vertex index, x and y coordinates, and adds to vertex list via x_points & y_points 
 * Error checks to see if vertex_index is out of bounds of graph_size
 */
void VelocityConverter::graph_add_vertex(int vertex_index, float x_val, float y_val) {
	if(vertex_index >= graph_size) {
		std::cout << "Error: vertex index is not valid - exceeds graph size" << std::endl;
		return;
	}
	x_points[vertex_index] = x_val;
	y_points[vertex_index] = y_val;
}
/* Dixtra's shortest path algorithm that stores the shortest path steps
 * Algorithm from https://www.geeksforgeeks.org/printing-paths-dijkstras-shortest-path-algorithm/
 * Returns array of shortest distances based on vertex indicies
 */
void VelocityConverter::graph_dixtra(int source_index, int parent_path[], float dist[]) {
	bool sepSet[graph_size];
	/// initialize data structures
	for(int i = 0; i < graph_size; i++) {
		// set all distances to infinity except at the source
		dist[i] = std::numeric_limits<float>::infinity();
		sepSet[i] = false;
		parent_path[source_index] = -1;
	}
	/// set the source distance to be zero so it gets processed first
	dist[source_index] = 0;
	///
	int u;
	float min;
	for(int i = 0; i < (graph_size - 1); i++) {
		/// find the vertex with min_distance from source from available vertices
		/// set this index as u
		min = std::numeric_limits<float>::infinity();
  		for(int v = 0; v < graph_size; v++) {
      	if (sepSet[v] == false && dist[v] <= min) {
            min = dist[v], u = v; 	
			}
		}
		/// set value that we visited u = true
		sepSet[u] = true;
		/// update distance values adjacent to this vertex
		for(int v = 0; v < graph_size; v++) {
			/// if v is not visited already, there is not already an edge from u to v
			/// and the distance from src to u to v is smaller than current distance from
			/// src to vertex v, then update the value
			/// and...store vertex u in parent array at index v
			if(!sepSet[v] && graph[u][v] && (dist[v] > dist[u] + graph[u][v])) {
				parent_path[v] = u;
				dist[v] = dist[u] + graph[u][v];
			}
		}
	}
}
/* Finds a vertex based on x and y coordinates
 * If the point is within a certain threshold of this desired-value
 * then consider it as the vertex - return the vertex index
 */
int VelocityConverter::graph_find_vertex(float x, float y) {
	/// if any points are close enough in the graph (0.1 for now)
	/// we have found that point
	float closeness_thresh = 0.1;
	for(int i = 0; i < graph_size; i++) {
		if((abs(x_points[i] - x) < closeness_thresh ) && (abs(y_points[i] - y) < closeness_thresh)) {
			return i;
		}
	}
	return -1;
}
// Algorithm from https://www.geeksforgeeks.org/printing-paths-dijkstras-shortest-path-algorithm/
// Function to print shortest 
// path from source to j 
// using parent array 
void printPath(int parent[], int j) 
{ 
      
    // Base Case : If j is source 
    if (parent[j] == -1) 
        return; 
  
    printPath(parent, parent[j]); 
  
    printf("%d ", j); 
} 
// Algorithm from https://www.geeksforgeeks.org/printing-paths-dijkstras-shortest-path-algorithm/  
// A utility function to print  
// the constructed distance 
// array 
int printSolution(int src, float dist[], int n,  
                      int parent[]) 
{ 
    printf("Vertex\t Distance\tPath"); 
    for (int i = 0; i < n; i++) 
    { 
        printf("\n%d -> %d \t\t %f\t\t%d ", 
                      src, i, dist[i], src); 
        printPath(parent, i); 
    } 
} 
/* localization gets the current point and desired point
 * Uses dixtra's algorithm to find the shortest path between them
 * Uses a recursive helper to segment the velocities
 */
void VelocityConverter::localization(geometry_msgs::Point desired_point) {
	/// find the vertex on the graph that corresponds to both:
	/// the initial position
	int src = graph_find_vertex(current_pose.x, current_pose.y);
	/// the final position
	int fin = graph_find_vertex(desired_point.x, desired_point.y);
	/// declare path that velocities will use
	int parent[graph_size] = {0};
	/// find shortest paths
	float distances[graph_size];
	/// calculate dixtra's algorithm
	graph_dixtra(src, parent, distances);
	/*
 	 *	PRINTING FOR DEBUGGING
    */
	printSolution(src, distances, graph_size, parent);
	std::cout << std::endl << std::endl  << "----- graph of distances -----" << std::endl;
	for(int i = 0; i < graph_size; i++) {
		for(int j = 0; j < graph_size; j++) {
			std::cout << graph[i][j] << "\t";
		}	
		std::cout << std::endl;
	}
	std::cout << std::endl << "----- x_points,y_points for duckietown -----" << std::endl;
	for(int i = 0; i < graph_size; i++) {
		std::cout << "( " << x_points[i] << ", " << y_points[i] << " )" << std::endl;
	}
	std::cout << std::endl << "To go from location: " << "index - " 
				 << src << ": (" << x_points[src] << ", " << y_points[src]
				 << ") to index - " << fin << ": (" << x_points[fin] 
       		 << ", " << y_points[fin] << ") ....." << std::endl;
	/*
 	 *	PRINTING FOR DEBUGGING
    */
	/// call recursive algorithm to control velocities
	localizationHelper(parent, fin);
}

/* localizationHelper is a recursive alrogithm to segment velocity controls
 * Takes in an array of parents to the shortest path and controls the duckiebot to each point
 * Uses a recursive helper to segment the velocities
 */
void VelocityConverter::localizationHelper(int parent[], int u) {
	/// base case - path is at source node (current_pose)
	if(parent[u] == -1) {
		return;
	}
	/// ---------- NEED TO INITILIAZE THESE PARAMETERS ----------
	/// control what area around the desired point where the duckiebot should stop
	float diff_threshold = 0.1;
	/// how powerful the linear velocity should be based on a specific distance
	float v_gain = 1.0;
	/// how powerful the angular velocity should be based on a specific distance 
	float omega_gain = 1.0;
	/// parameters based on the distance the duckiebot is told to actually turn
	/// These need to actually be measured based on the duckietown turn/intersection tiles
	float x_turn_threshold = 0.1;
	float y_turn_threshold = 0.1; 
	/// Declare publisher variable
	duckietown_msgs::Twist2DStamped velocity_command;
	velocity_command.v = 0;
	velocity_command.omega = 0;	

	/// CURRENT desired point is vertex u
	geometry_msgs::Point desired_point;
	desired_point.x = x_points[u];
	desired_point.y = y_points[u];
	
	/// if difference between desired_point and current point is big enough
	if(pow( (pow(desired_point.x,2) + pow(desired_point.y,2)),0.5) - pow((pow(current_pose.x,2) + pow(current_pose.y,2)),0.5) > diff_threshold) {
		/// calculate current velocity is based off how far away the object is
		velocity_command.v = v_gain * lin_dist(current_pose.x, desired_point.x, current_pose.y, desired_point.y);
		/// test if the duckiebot needs to turn
		/// determine if it needs to make a left or right turn based on where the actual desired point is relative to current_pose
		/// x_turn_threshold and y_turn_threshold control this a lot
		if(((desired_point.x - current_pose.x) >= x_turn_threshold) && ((desired_point.y - current_pose.y) >= y_turn_threshold)) {
			// if current_pose = horizontal		
			velocity_command.omega = omega_gain * (current_pose.theta - (M_PI / 2));
		} else if(((desired_point.x - current_pose.x) <= (-1 * x_turn_threshold)) && ((desired_point.y - current_pose.y) <= (-1 * y_turn_threshold))) {
			velocity_command.omega = omega_gain * (current_pose.theta - (M_PI / 2));
		} else if(((desired_point.x - current_pose.x) >= x_turn_threshold) && ((desired_point.y - current_pose.y) <= (-1 * y_turn_threshold))) {
			velocity_command.omega = omega_gain * (current_pose.theta + (M_PI / 2));
		} else if( ((desired_point.x - current_pose.x) <= (-1 * x_turn_threshold)) && ((desired_point.y - current_pose.y) >= y_turn_threshold)) {

			velocity_command.omega = omega_gain * (current_pose.theta + (M_PI / 2));
		} else {
			velocity_command.omega;
		}
		/// if the car is horizontal, negate the omega speed
		if( (abs(current_pose.theta) < (M_PI / 6)) || (abs(current_pose.theta) > ((5 * M_PI) / 6)) ) {
			velocity_command.omega = -1 * velocity_command.omega;
		}
	/// if the difference between the desired point and current point is small enough
	} else {
		/// set the velocity equal to 0
		velocity_command.v = 0;
	}

	/// recursive call to complete next steps of mission
	localizationHelper(parent, parent[u]);

	std::cout << "publish velocity: v = " << velocity_command.v << ", omega = " << velocity_command.omega << std::endl;
	/// publish the velocities
	velocity_publisher.publish(velocity_command);

	/// latch time until duckiebot completes its mission
	while((desired_point.x - current_pose.x > 0.1) && (desired_point.y - current_pose.y > 0.1));
}
/// Calculates the linear distance between two points: (xi, yi) to (xf, yf)
float VelocityConverter::lin_dist(float xi, float xf, float yi, float yf) {
	return pow((pow((xf - xi),2) + pow((yf - yi),2)), 0.5);
}
