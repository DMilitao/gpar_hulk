#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>
#include <std_msgs/String.h>
#include "geometry_msgs/Twist.h"
#include <string>
#include <nav_msgs/Odometry.h>

//Position
double x = 0;
double y = 0;
double theta = 0;

//Velocity
double vx = 0;
double vy = 0;
double w = 0;	


void callback(const turtlesim::Pose::ConstPtr& msg){
	x = msg->x;
	y = msg->y;
	theta = msg->theta;

	vx = msg->linear_velocity; //Turtlesim only has V_x velocity
	w = msg->angular_velocity;	

	static tf::TransformBroadcaster br; //Create a tf object to share the frame

	tf::Transform transform; //Our turtle frame (body frame)

	transform.setOrigin(tf::Vector3(x,y,0)); //change the body frame
	tf::Quaternion q; //orientation
	q.setRPY(0,0,theta); //change ROLL PITCH YAW to quaternion
	transform.setRotation(q);

	br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"odom","car")); //Send the body frame
	//Odom --> world frame
	//Car --> body frame
}

int main(int argc, char** argv){
	ros::init(argc,argv,"tf_odometry");

	ros::NodeHandle n;
	//nav_msg::Odometry is the msg we use to plot odometry on RVIZ
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom",1000);

	//To receive the position of TurtleSim
	ros::Subscriber sub = n.subscribe("turtle1/pose",1000,&callback);

	ros::Rate loop_rate(20);

	ros::Time current_time;

	while(ros::ok()){
		current_time = ros::Time::now();

		geometry_msgs::Quaternion odom_quat;
		odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,theta);

		nav_msgs::Odometry odom;

		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";
		odom.child_frame_id = "car";		

		odom.pose.pose.position.x = x;
		odom.pose.pose.position.y = y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;
		
		odom.twist.twist.linear.x = vx;
		odom.twist.twist.linear.y = 0;
		odom.twist.twist.linear.z = 0;
		odom.twist.twist.angular.x = 0;
		odom.twist.twist.angular.y = 0;
		odom.twist.twist.angular.z = w;

		odom_pub.publish(odom);

		ros::spinOnce();

		loop_rate.sleep();
}		
	
	return 0;
}
