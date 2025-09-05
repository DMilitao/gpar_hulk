#include "string.h"

#include <chrono>
#include <iostream>
#include <thread>
#include <sstream>

#include "driver_HDC2450.h"
#include "ros/ros.h"
#include "serial/serial.h"
#include "../../serial_port_define.h"

int main(int argc, char **argv){
	Driver HULK;
	ros::init(argc,argv,"self_test");
	ros::NodeHandle n;
	std::string serial_port = serial_port_driver;
	std::stringstream ss;

	const float th_battery_min = 24;

	if ( !HULK.serial_open(serial_port) ){
		return -1;
	} 
	ROS_INFO("Starting SelfTest HULK");

	HULK.get_all();
	ss << "Battery voltage level: " << HULK.volt_bat() << " V -> "<< (HULK.volt_bat() > th_battery_min ? "Charged" : "Need charge");
	ROS_INFO(ss.str().c_str());

	if ( HULK.volt_bat() < th_battery_min ) {
		return -1;
	}

	auto duration_move = std::chrono::seconds(4);
	ROS_INFO("Moving forward");

	auto start_time = std::chrono::steady_clock::now();

	while (std::chrono::steady_clock::now() - start_time < duration_move) {
		HULK.set_speed(30,30);
		HULK.get_speed();
		ss.str("");
		ss << "Left: " << HULK.left_speed()<<" | Right: " << HULK.right_speed();
		ROS_INFO(ss.str().c_str());
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}

	ROS_INFO("Moving backward");

	start_time = std::chrono::steady_clock::now();
		
	while (std::chrono::steady_clock::now() - start_time < duration_move) {
		HULK.set_speed(-30,-30);
		HULK.get_speed();
		ss.str("");
		ss << "Left: " << HULK.left_speed()<<" | Right: " << HULK.right_speed();
		ROS_INFO(ss.str().c_str());
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}

	ROS_INFO("Clockwise rotation");
	start_time = std::chrono::steady_clock::now();
		
	while (std::chrono::steady_clock::now() - start_time < duration_move) {
		HULK.set_speed(30,-30);
		HULK.get_speed();
		ss.str("");
		ss << "Left: " << HULK.left_speed()<<" | Right: " << HULK.right_speed();
		ROS_INFO(ss.str().c_str());
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}

	ROS_INFO("Counterclockwise rotation");
	start_time = std::chrono::steady_clock::now();
		
	while (std::chrono::steady_clock::now() - start_time < duration_move) {
		HULK.set_speed(-30,30);
		HULK.get_speed();
		ss.str("");
		ss << "Left: " << HULK.left_speed()<<" | Right: " << HULK.right_speed();
		ROS_INFO(ss.str().c_str());
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}
	HULK.set_speed(0,0);
	HULK.get_all();

	ss.str("");
	ss << "Battery voltage level: " << HULK.volt_bat() << " V -> "<< (HULK.volt_bat() > th_battery_min ? "Charged" : "Need charge");
	ROS_INFO(ss.str().c_str());
	
	ROS_INFO("Finishing SelfTest HULK");
}

