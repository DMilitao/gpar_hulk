#include <iostream>
#include "serial/serial.h"
#include <sstream>
#include "string.h"
#include "ros/ros.h"
#include "driver_HDC2450.h"

int main(int argc, char **argv){
	Driver HULK;
	ros::init(argc,argv,"serial_test");
	ros::NodeHandle n;
	
	HULK.serial_open("/dev/ttyACM0");

	ros::Rate freq(10);
	while(ros::ok()){
		HULK.set_speed(300,-300);
		HULK.read_speed();
		std::cout<<HULK.read_ve()<<" "<<HULK.read_vd()<<std::endl;
		freq.sleep();
		
}

}

