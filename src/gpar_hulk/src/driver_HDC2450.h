#include "string.h"

#include <cstdio>
#include <iostream>
#include <stdlib.h>
#include <sstream>
#include <unistd.h>

#include "sensor_msgs/BatteryState.h"
#include "serial/serial.h"

struct DataBase{

    int left_speed_rpm_;
    int right_speed_rpm_;
    
    float current_right_;
    float current_left_;
    
    int temp_MCU_;
    int temp_motor1_;
    int temp_motor2_;
    
    float volt_internal_;
    float volt_battery_;
    float volt_output_;

};

class Driver
{
	public:		
		Driver();		

		/**
		 * \brief Open the serial port
		 * \param serial_port The desired port to connect
		 */
		void serial_open(std::string serial_port);

		/**
		 * \brief Set the RPM speed of each wheel
		 * \param right_speed_rpm The speed of the right wheel
		 * \param left_speed_rpm The speed of the left wheel
		 */
		void set_speed(int right_speed_rpm, int left_speed_rpm);

		/**
		 * \brief Read the RPM speed of each wheel
		 */
		void get_speed();

		/**
		 * \brief Ger the RPM speed of left wheel
		 * \return The speed
		 */
		int left_speed();

		/**
		 * \brief Get the RPM speed of right wheel
		 * \return The speed
		 */
		int right_speed();	

		/**
		 * \brief Read the current each motor
		 */
		void get_current();

		/**
		 * \brief Get the current of right motor
		 * \return The current
		 */
		float current_right();

		/**
		 * \brief Get the current of left motor
		 * \return The current
		 */
		float current_left();

		/**
		 * \brief Read the temperature of components
		 */
		void get_temp();

		/**
		 * \brief Get the temperature of MCU
		 * \return The temperature
		 */
		int temp_MCU();

		/**
		 * \brief Get the temperature of right motor
		 * \return The temperature
		 */
		int temp_motor1();

		/**
		 * \brief Get the temperature of left motor
		 * \return The temperature
		 */
		int temp_motor2();

		/**
		 * \brief Read the voltage of components
		 */
		void get_volt();

		/**
		 * \brief Get the internal voltage
		 * \return The internal voltage
		 */
		float volt_int();

		/**
		 * \brief Get the battery voltage
		 * \return The battery voltage
		 */
		float volt_bat();

		/**
		 * \brief Get the output voltage
		 * \return The output voltage
		 */
		float volt_out();

		/**
		 * \brief Read all components
		 */
		void get_all();
		
		~Driver();

	private:
		DataBase data_;					//!> Storage data about system
		serial::Serial *serial_port_;	//!> Serial port object
		int cte_driver_ = 5;			//!> Driver constant for speed setpoint conversion
};

Driver::Driver(){
	std::cout<<"---HULK PRINCIPAL NODE--"<<std::endl;
}

void Driver::serial_open(std::string serial_port){
	serial_port_ = new serial::Serial(serial_port,115200,serial::Timeout::simpleTimeout(1000));

	while(!serial_port_->isOpen()) {
		std::cout<<"Waiting...";
	}

	std::cout<<"Serial port is ready!"<<std::endl;
}

void Driver::set_speed(int right_speed_rpm, int left_speed_rpm){	
	std::string msg;
	std::stringstream command1, command2;

	// The RPM speed needs to be converted in a range between -1000 and 1000 due to the driver
	// Knowing the maximal speed set in the driver is 200 RPM
	// The speed sent to driver should be expressed as (value/200RPM)*1000RPM or value*5

	// Motor 1 - Right Wheel || Motor 2 - Left Wheel
	command1<<"!G 1 "<<right_speed_rpm*cte_driver_<<"\r";
	serial_port_->write(command1.str());

	// Discard the first read (it is an echo of the command)
	msg = serial_port_->readline(100,"\r");
	msg = serial_port_->readline(100,"\r");

	command2<<"!G 2 "<<left_speed_rpm*cte_driver_<<"\r";
	serial_port_->write(command2.str());	

	// Discard the first read (it is an echo of the command)	
	msg = serial_port_->readline(100,"\r");
	msg = serial_port_->readline(100,"\r");
}

void Driver::get_speed(){
	std::string resposta;
	
	serial_port_->write("?S\r");

	// Discard the first read (it is an echo of the command)
	resposta = serial_port_->readline(100,"\r");
	resposta = serial_port_->readline(100,"\r");
	
	sscanf(resposta.c_str(),"S=%d:%d\r",&data_.right_speed_rpm_,&data_.left_speed_rpm_);
}

int Driver::left_speed(){
	return data_.left_speed_rpm_;
}			

int Driver::right_speed(){
    return data_.right_speed_rpm_;
}

void Driver::get_current(){
	std::string resposta;
	
	serial_port_->write("?A\r");
	
	// Discard the first read (it is an echo of the command)
	resposta = serial_port_->readline(100,"\r");
	resposta = serial_port_->readline(100,"\r");
	
	sscanf(resposta.c_str(),"A	=%f:%f\r",&data_.current_right_,&data_.current_left_);
}

float Driver::current_right(){
	return data_.current_right_;
}

float Driver::current_left(){
	return data_.current_left_;
}

void Driver::get_temp(){
	std::string resposta;
	
	serial_port_->write("?T\r");
	
	// Discard the first read (it is an echo of the command)
	resposta = serial_port_->readline(100,"\r");
	resposta = serial_port_->readline(100,"\r");
	
	sscanf(resposta.c_str(),"T=%d:%d:%d\r",&data_.temp_MCU_,&data_.temp_motor1_,&data_.temp_motor2_);
}

int Driver::temp_MCU(){
	return data_.temp_MCU_;
}

int Driver::temp_motor1(){
	return data_.temp_motor1_;
}

int Driver::temp_motor2(){
	return data_.temp_motor2_;
}	

void Driver::get_volt(){
	std::string resposta;
	
	serial_port_->write("?V\r");
	
	// Discard the first read (it is an echo of the command)
	resposta = serial_port_->readline(100,"\r");
	resposta = serial_port_->readline(100,"\r");
	
	sscanf(resposta.c_str(),"V=%f:%f:%f\r",&data_.volt_internal_,&data_.volt_battery_,&data_.volt_output_);
}

float Driver::volt_int(){
	return data_.volt_internal_/10;
}

float Driver::volt_bat(){
	return data_.volt_battery_/10;
}

float Driver::volt_out(){
	return data_.volt_output_/1000;
}
	

void Driver::get_all(){
	get_speed();
	get_current();
	get_temp();
	get_volt();
}

Driver::~Driver(){
	std::cout<<"\nFinish program..."<<std::endl;
	delete serial_port_;
}



	

