#include "ros/ros.h"
#include <sstream>
#include "std_msgs/String.h"
#include "serial/serial.h"
#include "geometry_msgs/Twist.h"
#include "driver_HDC2450.h"
#include "sensor_msgs/BatteryState.h"
#include "math.h"
#include "geometry_msgs/Point.h"

#include "../../serial_port_define.h"

void set_speed_rpm(const geometry_msgs::Twist::ConstPtr& velocidade);
//void Set_Position(const std_msgs::String::ConstPtr& position);
void hulk_battery();
void hulk_odometry();

const float PI = 3.141592654;
const float L = 0.42;  // Distance between wheels
const float R = 0.075; // Wheel radius
const float th_battery_min = 24;

float right_speed_rad;
float left_speed_rad;
int right_speed_rpm = 0;
int left_speed_rpm = 0;

//Orientation and Translation
float x = 0;
float y = 0;
float theta = 0;

std::string serial_port = serial_port_driver;

Driver HULK;

sensor_msgs::BatteryState hulk_data;

int main(int argc, char **argv)
{
	geometry_msgs::Point hulk_position;
	geometry_msgs::Point hulk_speed; //left_speed_rpm --> x and right_speed_rpm --> y

	ros::init(argc, argv, "hulk_node");
	ros::NodeHandle n;
	ros::NodeHandle n_private("~");

	ros::Publisher info_velocity = n_private.advertise<geometry_msgs::Point>("read_speed",1000);
	ros::Publisher info_battery = n_private.advertise<sensor_msgs::BatteryState>("battery_info",1000);
	ros::Publisher info_odometry = n_private.advertise<geometry_msgs::Point>("odometry",1000);

	ros::Subscriber get_velocity = n.subscribe("/speed",1000,set_speed_rpm);

	n_private.getParam("serial_port_driver",serial_port);	
	
	if ( !HULK.serial_open(serial_port) ){
		ros::shutdown();
	};
	
	ros::Rate freq(20);
	ros::Time current_time;

	while(ros::ok()){
	current_time = ros::Time::now();

	//if ( HULK.serial_isOpen() ) {
	//	ROS_FATAL("SERIAL COMMUNICATION FAILED");
	//	ros::shutdown();
	//	continue;
	//}

	HULK.get_all();
	hulk_battery();
	hulk_odometry();

	if ( hulk_data.voltage < th_battery_min ) {
		ROS_FATAL("Battery level extremely low: Shutting down the node");
		ros::shutdown();
		continue;
	}

	hulk_position.x = x;
	hulk_position.y = y;
	hulk_position.z = theta;

    hulk_speed.x = HULK.left_speed();
    hulk_speed.y = HULK.right_speed();
  
	info_odometry.publish(hulk_position);
	info_velocity.publish(hulk_speed);
	info_battery.publish(hulk_data);

	ros::spinOnce();
	
	freq.sleep();
	}

return 0;
}

//Realizando o cáculo das velocidades de cada motor em rpm
void set_speed_rpm(const geometry_msgs::Twist::ConstPtr& speed){
	float v = speed->linear.x;
	float w = speed->angular.z;
		
	right_speed_rad = (2*v+w*L)/(2*R);
	left_speed_rad = (2*v-w*L)/(2*R);

	right_speed_rpm = (right_speed_rad*60/(2*PI));
	left_speed_rpm = (left_speed_rad*60/(2*PI));
	
	HULK.set_speed(right_speed_rpm,left_speed_rpm);
}
    
void hulk_battery(){
	hulk_data.voltage = HULK.volt_bat();
	hulk_data.current = (HULK.current_right()*1000+HULK.current_left()*10);
	hulk_data.percentage = (HULK.volt_bat()/24)*100.0;
	
}
	
void hulk_odometry(){
	float left_speed_m = 0;
	float right_speed_m = 0;
	float dt = 1/20.0;

	// recebendo as velocidades das rodas em rpm
	left_speed_rpm = HULK.left_speed();
	right_speed_rpm = HULK.right_speed();
	// dt = 1/f ; f=20Hz

	// conversão para rad/s
	left_speed_rad = (left_speed_rpm*2*PI)/60;
	right_speed_rad = (right_speed_rpm*2*PI)/60;
	
	// conversão para m/s
	left_speed_m = left_speed_rad*R;
	right_speed_m = right_speed_rad*R;

	// cálculo de theta
	// obs: curva para direita -> w<0 ; curva para esquerda -> w>0
	theta = theta + ((right_speed_m - left_speed_m)/L)*dt;
	theta = atan2(sin(theta),cos(theta));

	// cálculo de X e Y
	x = x + ((left_speed_m + right_speed_m)/2)*cos(theta)*dt;
	y = y + ((left_speed_m + right_speed_m)/2)*sin(theta)*dt;
}	
