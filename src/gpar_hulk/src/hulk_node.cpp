#include "ros/ros.h"
#include <sstream>
#include "std_msgs/String.h"
#include "serial/serial.h"
#include "geometry_msgs/Twist.h"
#include "driver_HDC2450.h"
#include "sensor_msgs/BatteryState.h"
#include "math.h"
#include "geometry_msgs/Point.h"

void speed_motor_rpm(const geometry_msgs::Twist::ConstPtr& velocidade);
//void Set_Position(const std_msgs::String::ConstPtr& position);
void Dados_hulk();
void hulk_odometria();

const float PI = 3.141592654;
const float L = 0.42;  // Distance between wheels
const float R = 0.075; // Wheel radius
float right_speed_rad;
float left_speed_rad;
int right_speed_rpm = 0;
int left_speed_rpm = 0;

//Orientation and Translation
float x = 0;
float y = 0;
float theta = 0;

std::string porta = "/dev/ttyACM0";

Driver HULK;

sensor_msgs::BatteryState hulk_dados;

int main(int argc, char **argv)
{
	std_msgs::String velocidade;
	std_msgs::String position;
	geometry_msgs::Point odom;
	geometry_msgs::Point speed_hulk; //left_speed_rpm --> x and right_speed_rpm --> y

	ros::init(argc,argv,"hulk_node");
	ros::NodeHandle n;
	ros::NodeHandle n_private("~");

	ros::Publisher info_velocity = n_private.advertise<geometry_msgs::Point>("read_speed",1000);
	ros::Publisher info_battery = n_private.advertise<sensor_msgs::BatteryState>("battery_info",1000);
	ros::Publisher info_odom = n_private.advertise<geometry_msgs::Point>("odometria",1000);

	ros::Subscriber get_velocity = n.subscribe("/hulk_keyboard/speed",1000,speed_motor_rpm);
	ros::Subscriber get_velocity_self_test = n.subscribe("/hulk_self_test/speed",1000,speed_motor_rpm);

	n_private.getParam("porta_serial",porta);	
	
	HULK.serial_open(porta);
	
	ros::Rate freq(20);

	ros::Time current_time;

	while(ros::ok()){
	current_time = ros::Time::now();

	HULK.read();
	Dados_hulk();
	hulk_odometria();

	odom.x = x;
	odom.y = y;
	odom.z = theta;

    speed_hulk.x = HULK.read_ve();
    speed_hulk.y = HULK.read_vd();
  
	info_odom.publish(odom);
	info_velocity.publish(speed_hulk);
	info_battery.publish(hulk_dados);

	ros::spinOnce();
	
	freq.sleep();
	}

return 0;
}

//Realizando o cáculo das velocidades de cada motor em rpm
void speed_motor_rpm(const geometry_msgs::Twist::ConstPtr& speed){
	float v = speed->linear.x;
	float w = speed->angular.z;
		
	right_speed_rad = (2*v+w*L)/(2*R);
	left_speed_rad = (2*v-w*L)/(2*R);

	right_speed_rpm = (right_speed_rad*60/(2*PI));
	left_speed_rpm = (left_speed_rad*60/(2*PI));
	
	std::cout<<"Velocidade roda direita = "<<right_speed_rpm<<"\nVelocidade roda esquerda = "<<left_speed_rpm<<std::endl;
	HULK.set_speed(right_speed_rpm,left_speed_rpm);
}
    
void Dados_hulk(){
	
	hulk_dados.voltage = HULK.read_volt_bat();
	hulk_dados.current = (HULK.read_current_d()*1000+HULK.read_current_e()*10);
	hulk_dados.percentage = (HULK.read_volt_bat()/24)*100.0;
	
}
	
void hulk_odometria(){
	float left_speed_m = 0;
	float right_speed_m = 0;
	float dt = 1/20.0;

	// recebendo as velocidades das rodas em rpm
	left_speed_rpm = HULK.read_ve();
	right_speed_rpm = HULK.read_vd();
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

	// cálculo de X e Y
	x = x + ((left_speed_m + right_speed_m)/2)*cos(theta)*dt;
	y = y + ((left_speed_m + right_speed_m)/2)*sin(theta)*dt;
}	
	


