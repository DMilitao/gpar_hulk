#include "ros/ros.h"
#include <sstream>
#include "std_msgs/String.h"
#include "serial/serial.h"
#include "geometry_msgs/Twist.h"
#include "driver_HDC2450.h"
#include "sensor_msgs/BatteryState.h"
#include "math.h"
#include "geometry_msgs/Point.h"

double linear, angular;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "hulk_trajectory");
	ros::NodeHandle n;

	geometry_msgs::Twist speed;
	ros::Publisher set_speed = n.advertise<geometry_msgs::Twist>("speed", 1000);
	ros::Subscriber get_position = n.subscribe("/odometry", 1000, position_control_law);

	ros::Rate freq(20);
	while(ros::ok()){

		speed.linear.x = linear;
		speed.angular.z = angular;

		set_speed.publish(speed);
		
		ros::spinOnce();		
		freq.sleep();
	}

return 0;
}

void position_control_law(const geometry_msgs::Point::ConstPtr& position){
	std::stringstream ss;
	ss << "Odometry: X " << position.x << " | Y " << position.y << " | Theta " << position.z;

	ROS_INFO(ss.str().c_str);
	linear = 0;
	angular = 0;
}

