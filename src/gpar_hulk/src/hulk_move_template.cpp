#include "ros/ros.h"
#include <sstream>
#include "std_msgs/String.h"
#include "serial/serial.h"
#include "geometry_msgs/Twist.h"
#include "driver_HDC2450.h"
#include "sensor_msgs/BatteryState.h"
#include "math.h"
#include "geometry_msgs/Point.h"


int main(int argc, char **argv)
{
	//std_msgs::String velocidade;
	//std_msgs::String position;
	geometry_msgs::Point hulk_position;
	geometry_msgs::Point hulk_speed; //left_speed_rpm --> x and right_speed_rpm --> y

	ros::init(argc, argv, "hulk_node");
	ros::NodeHandle n;
	ros::NodeHandle n_private("~");

	ros::Publisher info_velocity = n_private.advertise<geometry_msgs::Point>("read_speed",1000);
	ros::Publisher info_battery = n_private.advertise<sensor_msgs::BatteryState>("battery_info",1000);
	ros::Publisher info_odometry = n_private.advertise<geometry_msgs::Point>("odometry",1000);

	ros::Subscriber get_velocity = n.subscribe("/hulk_keyboard/speed",1000,set_speed_rpm);

	ros::Rate freq(20);

	while(ros::ok()){


		ros::spinOnce();
		
		freq.sleep();
	}

return 0;
}


