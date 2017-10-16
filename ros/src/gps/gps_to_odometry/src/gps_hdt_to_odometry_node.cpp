/****************************************************************************
# Copyright (c) 2016, author Mikkel Kragh Hansen
# mkha@eng.au.dk
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in the
#   documentation and/or other materials provided with the distribution.
# * Neither the name FroboMind nor the
#   names of its contributors may be used to endorse or promote products
#   derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ****************************************************************************/


#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

//#include "htf_safe_msgs/Course_Speed.h"
//#include "htf_rtk_gps/GNSSPositionData.h"
#include "htf_safe_msgs/GPHDT.h"

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>

#define PI 3.14159265359f
#define DEGREE_TO_RADIANS 2*PI/360

ros::Publisher pubOdometry, pubIMU;

std::string parent_frame, child_frame;
double covariance_yaw;

void gps_GPHDT_handler(const htf_safe_msgs::GPHDTConstPtr msg_hdt)
{
	// Orientation
	const double roll = 0.0, pitch = 0.0;
	const double yaw = 2*PI-msg_hdt->HEADING_DEGREES*DEGREE_TO_RADIANS-PI/2;

	tf::Quaternion q;
	q.setRPY(roll, pitch, yaw);
	geometry_msgs::Quaternion orientation;
	tf::quaternionTFToMsg(q, orientation);

	// Position
	geometry_msgs::Point position;
	position.x = 0;
	position.y = 0;
	position.z = 0; // Not used for 2D odometry
	
	geometry_msgs::Twist twist;
	//twist.linear.x = msg_gps_course_speed->SPEED_OVER_GROUND;

	// Pose
	geometry_msgs::Pose pose;
	pose.position = position;
	pose.orientation = orientation;

	geometry_msgs::PoseWithCovariance poseWithCovariance;
	poseWithCovariance.pose = pose;
	geometry_msgs::TwistWithCovariance twistWithCovariance;
	twistWithCovariance.twist = twist;
//	float covariance[36];
	boost::array<float, 36> covariance;
	std::fill( covariance.begin(), covariance.begin() + covariance.size(), 0 );
	for (int i=0;i<6;i++)
		covariance[i+i*6]=1.0f;

	poseWithCovariance.covariance = covariance;
	twistWithCovariance.covariance = covariance;

	nav_msgs::Odometry msg_odometry;
	msg_odometry.header = msg_hdt->header;
	msg_odometry.header.frame_id = parent_frame;
	msg_odometry.child_frame_id = child_frame;
	msg_odometry.pose = poseWithCovariance;
	msg_odometry.twist = twistWithCovariance;

	pubOdometry.publish(msg_odometry);

	// IMU
	boost::array<float, 9> orientation_covariance;
	std::fill( orientation_covariance.begin(), orientation_covariance.begin() + orientation_covariance.size(), 0 );
	orientation_covariance.at(0 + 0 * 3) = 1000.0f;
	orientation_covariance.at(1 + 1 * 3) = 1000.0f;
	orientation_covariance.at(2 + 2 * 3) = covariance_yaw;
	//	for (int i=0;i<3;i++)
//		orientation_covariance.at(i + i * 3) = 1.0f;
	geometry_msgs::Vector3 angular_velocity;
	angular_velocity.x = 0;
	angular_velocity.y = 0;
	angular_velocity.z = 0;
	boost::array<float, 9> angular_velocity_covariance;
	std::fill( angular_velocity_covariance.begin(), angular_velocity_covariance.begin() + angular_velocity_covariance.size(), 0 );
	for (int i=0;i<3;i++)
		angular_velocity_covariance.at(i + i * 3) = 1000.0f;
	geometry_msgs::Vector3 linear_acceleration;
	linear_acceleration.x = 0; // cos(yaw);
	linear_acceleration.y = 0; //-sin(yaw);
	linear_acceleration.z = 0;

	sensor_msgs::Imu msg_imu;
	msg_imu.header = msg_hdt->header;
	msg_imu.header.frame_id = "RTK_GPS_HEADING";
	msg_imu.orientation = orientation;
	msg_imu.orientation_covariance = orientation_covariance;
	msg_imu.angular_velocity = angular_velocity;
	msg_imu.angular_velocity_covariance = angular_velocity_covariance;
	msg_imu.linear_acceleration = linear_acceleration;


	pubIMU.publish(msg_imu);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "htf_rtk_gps_to_odometry_node");
	ros::NodeHandle n;
	ros::NodeHandle private_n("~");


	ROS_INFO("Started htf_rtk_gps_to_odometry_node");

	private_n.param<std::string>("parent_frame", parent_frame, "odom");
	private_n.param<std::string>("child_frame", child_frame, "RTK_GPS_HEADING");
	private_n.param<double>("covariance_yaw", covariance_yaw, 0.0f);

	ros::Subscriber sub_gps_GPHDT = n.subscribe<htf_safe_msgs::GPHDT>("gps/GPHDT", 1, gps_GPHDT_handler);
	pubOdometry = n.advertise<nav_msgs::Odometry>("gps/odometry", 1);
	pubIMU = n.advertise<sensor_msgs::Imu>("gps/imu", 1);

	ros::spin();

	return 0;
}
