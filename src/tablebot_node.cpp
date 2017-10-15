#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float32.h>
#include <math.h>
#include <tablebot_pkg/MotorState.h>
#include "tablebot_node.h"

ros::Publisher robotCommandPub;
ros::Publisher robotRelativeCommandPub;
ros::Publisher robotRelativeAngularSpeedPub;
ros::Publisher robotRelativeLinearSpeedPub;

bool foundEdge = false;
double robotYaw = 0.0;
bool robotIdle = true;
double lineRho = 0.0;
double lineTheta = 0.0;
bool foundLine = false;

enum RobotState {Forward, Stop, SpinStart, SpinWait};

void pointCloudCallBack(const sensor_msgs::PointCloud2ConstPtr& cloud) {
	ROS_INFO("[PointCloud CallBack] height = %d", cloud->height);
	ROS_INFO("[PointCloud CallBack] width = %d", cloud->width);

	if (cloud->is_bigendian) {
		ROS_INFO("[PointCloud CallBack] is_bigendian = true");		
	}
	else {
		ROS_INFO("[PointCloud CallBack] is_bigendian = false");
	}

	ROS_INFO("[PointCloud CallBack] point_step = %d", cloud->point_step);
	ROS_INFO("[PointCloud CallBack] row_step = %d", cloud->row_step);

	if (cloud->is_dense) {
		ROS_INFO("[PointCloud CallBack] is_dense = true");		
	}
	else {
		ROS_INFO("[PointCloud CallBack] is_dense = false");
	}
}

void odometryCallBack(const nav_msgs::Odometry::ConstPtr odom) {
	tf::Quaternion q(
		odom->pose.pose.orientation.x,
		odom->pose.pose.orientation.y,
		odom->pose.pose.orientation.z,
		odom->pose.pose.orientation.w);
	tf::Matrix3x3 m(q);

	robotYaw = getYaw(q);
	// ROS_INFO("[Odometry] yaw = %f", robotYaw * 180.0 / M_PI);

}

void motorStateCallBack(const tablebot_pkg::MotorState::ConstPtr motorState) {
	if ( (motorState->state[Left] == "idle" || motorState->state[Left] == "holding") &&
		 (motorState->state[Right] == "idle" || motorState->state[Right] == "holding") ) {
		robotIdle = true;
	}
	else {
		robotIdle = false;
	}
}

void lineDetectCallBack(const geometry_msgs::Pose2D::ConstPtr lineDetect) {
	foundLine = true;
	lineRho = lineDetect->x;
	lineTheta = lineDetect->theta;
}



void depthImageCallBack(const sensor_msgs::ImageConstPtr& image) {
	// ROS_INFO("[DepthImage CallBack] height = %d", image->height);
	// ROS_INFO("[DepthImage CallBack] width = %d", image->width);
	// ROS_INFO_STREAM("[DepthImage CallBack] encoding = " << image->encoding);

	// if (image->is_bigendian) {
	// 	ROS_INFO("[DepthImage CallBack] is_bigendian = true");		
	// }
	// else {
	// 	ROS_INFO("[DepthImage CallBack] is_bigendian = false");
	// }

	// ROS_INFO("[DepthImage CallBack] step = %d", image->step);

	unsigned int max = 0;
	unsigned int min = 0xffff;
	unsigned long int avg = 0;
	int samples = 0;

	for (int y = 239; y < 240; y++) {
		for (int x = 0; x < 320; x++) {
			unsigned int index = y * (image->step) + 2*x;
			unsigned int point = (image->data[index]) |
								 ((image->data[index + 1] << 8) & 0xff00);
			if ( (point != 0) && (point != 65535) ) {
				if (point > max)
					max = point;
				if (point < min)
					min = point;
			}

			avg += point;
			samples++;

			// ROS_INFO("     %d", point);

		}
	}

	avg /= samples;

	geometry_msgs::Twist robotCommand;

	if (avg > 100) {
		foundEdge = true;
	}
	else {
		foundEdge = false;
	}

	ROS_INFO("[DepthImage CallBack] avg = %ld", avg);

	// ROS_INFO("[DepthImage CallBack] max = %d  min = %d", max, min);

}






int main (int argc, char** argv) {
	RobotState robotState = Forward;
	geometry_msgs::Twist robotCommand;
	double startingYaw = 0;
	double angleR;
	double turned;
	std_msgs::Float32 linearSpeed;
	std_msgs::Float32 angularSpeed;
	geometry_msgs::Pose2D relativeCommand;
	geometry_msgs::Pose2D stopCommand;

	// Initialize ROS
	ros::init (argc, argv, "tablebot_node");
	ros::NodeHandle nh;

	ros::Rate r(10); // 10 hz

	// Subscribe to PointCloud data
	// ros::Subscriber pointCloudSub = nh.subscribe ("/camera/depth/points", 1, pointCloudCallBack);

	// Subscribe to Depth Image
	//ros::Subscriber depthImageSub = nh.subscribe("/camera/depth/image_raw", 1, depthImageCallBack);

	// Subscribe to Odometry
	// ros::Subscriber odometrySub = nh.subscribe("/odom", 1, odometryCallBack);

	// Subscribe to Motor States
	ros::Subscriber motorStateSub = nh.subscribe("/ev3/motorStates", 1, motorStateCallBack);

	// Create a ROS publisher for robot Twist commands
	robotCommandPub = nh.advertise<geometry_msgs::Twist> ("/cmd_vel_mux/input/teleop", 1);

	// Subscribe to Line Detect messages
	ros::Subscriber lineDetectSub = nh.subscribe("/lineDetect", 1, lineDetectCallBack);

	// Create a ROS publishers for robot relative motion commands
	robotRelativeCommandPub = nh.advertise<geometry_msgs::Pose2D> ("/cmd_relative_move", 1);
	robotRelativeLinearSpeedPub = nh.advertise<std_msgs::Float32> ("/relative_linear_speed", 1);
	robotRelativeAngularSpeedPub = nh.advertise<std_msgs::Float32> ("/relative_angular_speed", 1);

	stopCommand.x = 0.0;
	stopCommand.y = 0.0;
	stopCommand.theta = 0.0;

	linearSpeed.data = 0.1;
	angularSpeed.data = 0.5;

	ros::Duration(10.0).sleep();

	robotRelativeLinearSpeedPub.publish(linearSpeed);
	robotRelativeAngularSpeedPub.publish(angularSpeed);
	ros::spinOnce();
	ros::Duration(0.5).sleep();

	robotRelativeCommandPub.publish(stopCommand);
	ros::spinOnce();
	ros::Duration(0.5).sleep();

	relativeCommand.x = 0.0;
	relativeCommand.y = 0.0;
	relativeCommand.theta = M_PI;


	bool done = false;

	while(ros::ok() && !done) {

		// Spin
		ros::spinOnce();

		robotCommand.linear.x = 0;
		robotCommand.linear.y = 0;
		robotCommand.linear.z = 0;
		robotCommand.angular.x = 0;
		robotCommand.angular.y = 0;
		robotCommand.angular.z = 0;		


		if (foundLine) {
			if (lineTheta > 1.02*M_PI/2) {
				robotCommand.angular.z = -0.5;
			}
			else if (lineTheta < 0.98*M_PI/2) {
				robotCommand.angular.z = 0.5;
			}
			else {
				robotCommand.angular.z = 0;		
			}
			foundLine = false;
			robotCommandPub.publish(robotCommand);
		}

		#if 0
		//
		// simple state machine version
		//
		switch(robotState) {
			case Forward:
				if (foundEdge) {
					robotState = Stop;
				}
				else {
					robotCommand.linear.x = 0.1;
				}
				robotCommandPub.publish(robotCommand);
				break;
			case Stop:
				if (robotIdle) {
					robotState = SpinStart;
					robotRelativeCommandPub.publish(relativeCommand);
				}
				else {
					robotState = Stop;
					robotRelativeCommandPub.publish(stopCommand);
				}
				break;
			case SpinStart:
				if (robotIdle) {
					robotState = SpinStart;
				}
				else {
					robotState = SpinWait;
				}
				break;
			case SpinWait:
				if (robotIdle) {
					robotState = Forward;
				}
				else {
					robotState = SpinWait;
				}
				break;
			default:
				robotState = Forward;
				break;
		}
		#endif

		r.sleep();


	}
}
