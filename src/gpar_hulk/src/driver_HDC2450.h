#include <iostream>
#include <sstream>
#include "string.h"
#include "serial/serial.h"
#include <unistd.h>
#include <stdlib.h>
#include <cstdio>
#include "sensor_msgs/BatteryState.h"

struct Leitura_String{

    int ve_rpm;
    int vd_rpm;
    
    float current_d;
    float current_e;
    
    int temp_MCU;
    int temp_motor1;
    int temp_motor2;
    
    float volt_internal;
    float volt_battery;
    float volt_output;

};

class Driver
{
	public:		
		Driver();		

		/**
		 * \brief Open the serial port
		 * \param porta The desired port to connect
		 */
		void serial_open(std::string porta);

		/**
		 * \brief Set the RPM speed of each wheel
		 * \param vd_rpm The speed of the right wheel
		 * \param ve_rpm The speed of the left wheel
		 */
		void set_speed(int vd_rpm, int ve_rpm);

		/**
		 * \brief Read the RPM speed of each wheel
		 */
		void read_speed();

		/**
		 * \brief Ger the RPM speed of left wheel
		 * \return The speed
		 */
		int read_ve();

		/**
		 * \brief Get the RPM speed of right wheel
		 * \return The speed
		 */
		int read_vd();	

		/**
		 * \brief Read the current each motor
		 */
		void read_current();

		/**
		 * \brief Get the current of right motor
		 * \return The current
		 */
		float read_current_d();

		/**
		 * \brief Get the current of left motor
		 * \return The current
		 */
		float read_current_e();

		/**
		 * \brief Read the temperature of components
		 */
		void read_temp();

		/**
		 * \brief Get the temperature of MCU
		 * \return The temperature
		 */
		int read_temp_MCU();

		/**
		 * \brief Get the temperature of right motor
		 * \return The temperature
		 */
		int read_temp_motor1();

		/**
		 * \brief Get the temperature of left motor
		 * \return The temperature
		 */
		int read_temp_motor2();

		/**
		 * \brief Read the voltage of components
		 */
		void read_volt();

		/**
		 * \brief Get the internal voltage
		 * \return The internal voltage
		 */
		float read_volt_int();

		/**
		 * \brief Get the battery voltage
		 * \return The battery voltage
		 */
		float read_volt_bat();

		/**
		 * \brief Get the output voltage
		 * \return The output voltage
		 */
		float read_volt_out();

		/**
		 * \brief Read all components
		 */
		void read();
		
		~Driver();

	private:
		Leitura_String dados;			//!> Storage data about system
		serial::Serial *porta_serial;	//!> Serial port object
	
};

Driver::Driver(){
	std::cout<<"---HULK PRINCIPAL NODE--"<<std::endl;
}

void Driver::serial_open(std::string porta){
	porta_serial = new serial::Serial(porta,115200,serial::Timeout::simpleTimeout(1000));

	while(!porta_serial->isOpen()) {
		std::cout<<"Waiting...";
	}

	std::cout<<"Serial port is ready!"<<std::endl;
}

void Driver::set_speed(int vd_rpm, int ve_rpm){	
	std::string msg;
	std::stringstream comando1, comando2;

	// Motor 1 - Right Wheel || Motor 2 - Left Wheel
	comando1<<"!G 1 "<<vd_rpm<<"\r";
	porta_serial->write(comando1.str());

	// Discard the first read (it is an echo of the command)
	msg = porta_serial->readline(100,"\r");
	msg = porta_serial->readline(100,"\r");

	comando2<<"!G 2 "<<ve_rpm<<"\r";
	porta_serial->write(comando2.str());	

	// Discard the first read (it is an echo of the command)	
	msg = porta_serial->readline(100,"\r");
	msg = porta_serial->readline(100,"\r");
}

void Driver::read_speed(){
	std::string resposta;
	
	porta_serial->write("?S\r");

	// Discard the first read (it is an echo of the command)
	resposta = porta_serial->readline(100,"\r");
	resposta = porta_serial->readline(100,"\r");
	
	sscanf(resposta.c_str(),"S=%d:%d\r",&dados.vd_rpm,&dados.ve_rpm);
}

int Driver::read_ve(){
	return dados.ve_rpm;
}			

int Driver::read_vd(){

        return dados.vd_rpm;
}	
void Driver::read_current(){
	std::string resposta;
	
	porta_serial->write("?A\r");
	
	// Discard the first read (it is an echo of the command)
	resposta = porta_serial->readline(100,"\r");
	resposta = porta_serial->readline(100,"\r");
	
	sscanf(resposta.c_str(),"A	=%f:%f\r",&dados.current_d,&dados.current_e);
}

float Driver::read_current_d(){
	return dados.current_d;
}

float Driver::read_current_e(){
	return dados.current_e;
}

void Driver::read_temp(){
	std::string resposta;
	
	porta_serial->write("?T\r");
	
	// Discard the first read (it is an echo of the command)
	resposta = porta_serial->readline(100,"\r");
	resposta = porta_serial->readline(100,"\r");
	
	sscanf(resposta.c_str(),"T=%d:%d:%d\r",&dados.temp_MCU,&dados.temp_motor1,&dados.temp_motor2);
}

int Driver::read_temp_MCU(){
	return dados.temp_MCU;
}

int Driver::read_temp_motor1(){
	return dados.temp_motor1;
}

int Driver::read_temp_motor2(){
	return dados.temp_motor2;
}	

void Driver::read_volt(){
	std::string resposta;
	
	porta_serial->write("?V\r");
	
	// Discard the first read (it is an echo of the command)
	resposta = porta_serial->readline(100,"\r");
	resposta = porta_serial->readline(100,"\r");
	
	sscanf(resposta.c_str(),"V=%f:%f:%f\r",&dados.volt_internal,&dados.volt_battery,&dados.volt_output);
}

float Driver::read_volt_int(){
	return dados.volt_internal/10;
}

float Driver::read_volt_bat(){
	return dados.volt_battery/10;
}

float Driver::read_volt_out(){
	return dados.volt_output/1000;
}
	

void Driver::read(){
	read_speed();
	read_current();
	read_temp();
	read_volt();
}

Driver::~Driver(){
	std::cout<<"\nFinish program..."<<std::endl;
	delete porta_serial;
}



	

