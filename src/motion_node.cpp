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
			// get the car number and do some string operations to get correct subscription/publisher topics
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
			
	private:
		// ros variables
		ros::NodeHandle nh;
		ros::Subscriber pose_subscriber;
		ros::Subscriber trajectory_subscriber;
		ros::Publisher velocity_publisher;

		// helper variables
		geometry_msgs::Pose2D current_pose;
		int carNum;

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
	int x_final = desired_point.x;
	int y_final = desired_point.x;
	duckietown_msgs::Twist2DStamped velocity_command;
	velocity_command.v = 0;
	velocity_command.omega = 0;

	/*********************************************************/
	/*************** Student Code Goes Here ******************/
	/*********************************************************/
	
	

	/*********************************************************/
	/*********************************************************/

	velocity_publisher.publish(velocity_command);
}
