#include <iostream>
#include "serial/serial.h"
#include <sstream>
#include "string.h"
#include "ros/ros.h"
#include "driver_HDC2450.h"

#include "../../serial_port_define.h"

int main(int argc, char **argv){
	Driver HULK;
	ros::init(argc,argv,"serial_test");
	ros::NodeHandle n;

	std::string serial_port = serial_port_driver;
	
	if ( !HULK.serial_open(serial_port) ){
		return -1;
	} 

	ros::Rate freq(10);
	while(ros::ok()){
		HULK.set_speed(20,-25);
		HULK.get_speed();
		std::stringstream ss;
		ss << HULK.left_speed()<<" "<<HULK.right_speed();
		ROS_INFO(ss.str().c_str());
		freq.sleep();		
	}
}

