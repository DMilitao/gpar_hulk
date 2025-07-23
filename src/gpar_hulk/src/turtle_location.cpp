#include <iostream>
#include <sstream>
#include "string.h"
#include "ros/ros.h"
#include <cmath>
#include <ctime>
#include <wiringPiI2C.h>
#include <wiringPi.h>
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

float x,x_;
float y,y_;
float theta,theta_;

double t,t0,dt;

void location(const geometry_msgs::Twist::ConstPtr& v){
	t = millis()/1000.0;
	dt = t - t0;
	theta = theta_ + v->angular.z*dt;
	x = x_ + v->linear.x*cos(theta);
	y = y_ + v->linear.x*sin(theta);
	t0 = t;
}

int main(int argc, char **argv){
	x_ = 5.544445;
	y_ = 5.544445;
	theta_ = 0;
	t0 = 0;
	std_msgs::String msg;

	ros::init(argc,argv,"turtle_location");
	ros::NodeHandle n;
	
	ros::Subscriber sub = n.subscribe("/turtle1/cmd_vel",1000,location);
	ros::Publisher pub = n.advertise<std_msgs::String>("location",1000);

	ros::Rate freq(10);
	while(ros::ok()){
		std::stringstream pos;

		pos<<"x = "<<x<<" y = "<<y<<" theta = "<<theta<<std::endl;

		msg.data = pos.str();
		
		pub.publish(msg);

		ros::spinOnce();
		freq.sleep();
		
}

}

