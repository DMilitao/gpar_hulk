#include "string.h"

#include <chrono>
#include <iostream>
#include <thread>
#include <sstream>

#include "driver_HDC2450.h"
#include "ros/ros.h"
#include "serial/serial.h"

int main(int argc, char **argv){
	Driver HULK;
	ros::init(argc,argv,"self_test");
	ros::NodeHandle n;
	
	const float th_battery_min = 24;


	HULK.serial_open("/dev/ttyACM0");
	std::cout<<" Starting SelfTest HULK " << std::endl;

	HULK.get_all();

	std::cout<< "Battery voltage level: " << HULK.volt_bat() << " V -> "<< (HULK.volt_bat() > th_battery_min ? "Charged" : "Need charge") << std::endl;

	if ( HULK.volt_bat() < th_battery_min ) {
		return -1;
	}

	auto duration_move = std::chrono::seconds(4);
	std::cout<<" Moving forward " <<std::endl;

	auto start_time = std::chrono::steady_clock::now();

	while (std::chrono::steady_clock::now() - start_time < duration_move) {
		HULK.set_speed(30,30);
		HULK.get_speed();
		std::cout << "Left: " << HULK.left_speed()<<" | Right: " << HULK.right_speed() << std::endl;
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}

	std::cout<<" Moving backward " <<std::endl;

	start_time = std::chrono::steady_clock::now();
		
	while (std::chrono::steady_clock::now() - start_time < duration_move) {
		HULK.set_speed(-30,-30);
		HULK.get_speed();
		std::cout << "Left: " << HULK.left_speed()<<" | Right: " << HULK.right_speed() << std::endl;
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}

	std::cout<<" Clockwise rotation " <<std::endl;
	start_time = std::chrono::steady_clock::now();
		
	while (std::chrono::steady_clock::now() - start_time < duration_move) {
		HULK.set_speed(30,-30);
		HULK.get_speed();
		std::cout << "Left: " << HULK.left_speed()<<" | Right: " << HULK.right_speed() << std::endl;
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}

	std::cout<<" Counterclockwise rotation " <<std::endl;
	start_time = std::chrono::steady_clock::now();
		
	while (std::chrono::steady_clock::now() - start_time < duration_move) {
		HULK.set_speed(-30,30);
		HULK.get_speed();
		std::cout << "Left: " << HULK.left_speed()<<" | Right: " << HULK.right_speed() << std::endl;
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}

	HULK.get_all();

	std::cout<< "Battery voltage level: " << HULK.volt_bat() << " V -> "<< (HULK.volt_bat() > th_battery_min ? "Charged" : "Need charge") << std::endl;	
	std::cout<< "Finishing SelfTest HULK " << std::endl;
}

