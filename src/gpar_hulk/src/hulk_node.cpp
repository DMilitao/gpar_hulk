#include "ros/ros.h"
#include <sstream>
#include "std_msgs/String.h"
#include "serial/serial.h"
#include "geometry_msgs/Twist.h"

#include "sensor_msgs/BatteryState.h"
#include "sensor_msgs/Imu.h"
#include "math.h"
#include "geometry_msgs/Point.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "nav_msgs/Odometry.h"

#include "mpu.h"
#include "driver_HDC2450.h"
#include "../../serial_port_define.h"

void init_msgs();
void publish_zero_rpm(const ros::TimerEvent& event);
void set_speed_rpm(const geometry_msgs::Twist::ConstPtr& velocidade);
void simplified_odometry(const nav_msgs::Odometry::ConstPtr& msg);
//void Set_Position(const std_msgs::String::ConstPtr& position);
void hulk_battery();
void hulk_odometry();

const float PI = 3.141592654;
const float L = 0.42;  // Distance between wheels
const float R = 0.075; // Wheel radius
const float th_battery_min = 24;

//Orientation and Translation
float x = 0;
float y = 0;
float theta = 0;

ros::Time last_time_odom;

float trans_speed = 0;
float theta_speed = 0;

std::string serial_port = serial_port_driver;

Driver HULK;
mpu mpu_1;

sensor_msgs::BatteryState hulk_data;
geometry_msgs::Point odom;
geometry_msgs::Point hulk_speed; //left_speed_rpm --> x and right_speed_rpm --> y
nav_msgs::Odometry hulk_position;
sensor_msgs::Imu hulk_imu;
tf2::Quaternion q;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "hulk_node");
	ros::NodeHandle n;
	ros::NodeHandle n_private("~");

    ros::Publisher zero_vel_pub = n_private.advertise<geometry_msgs::Twist>("cmd_vel_zero", 1);
	geometry_msgs::Twist zero_cmd;

	ros::Publisher info_velocity = n_private.advertise<geometry_msgs::Point>("read_speed", 1000);

	ros::Publisher info_battery = n_private.advertise<sensor_msgs::BatteryState>("battery_info", 1000);

	ros::Publisher info_odometry = n_private.advertise<nav_msgs::Odometry>("odometry", 1000);
	ros::Publisher info_odometry_simplified = n_private.advertise<geometry_msgs::Point>("odometry_filtered_simplifed", 1000);

	ros::Publisher info_mpu = n_private.advertise<sensor_msgs::Imu>("imu_raw", 1000);

	ros::Subscriber get_velocity = n.subscribe("/speed", 1000, set_speed_rpm);
	ros::Subscriber get_position = n.subscribe("/odometry/filtered", 1000, simplified_odometry);

	init_msgs();

	n_private.getParam("serial_port_driver",serial_port);	

	if ( !HULK.serial_open(serial_port) ){
		ROS_FATAL("SERIAL PORT UNAVAILABLE");
		ros::shutdown();
	};

	mpu_1.wakeUp();

	if ( !mpu_1.isConnected() ) {
		ROS_FATAL("MPU 1 UNAVAILABLE");
		ros::shutdown();
	}

	mpu_1.fix_bias();

	ros::Rate freq(20);
	ros::Time current_time;
	last_time_odom = ros::Time::now();

	while(ros::ok()){
		current_time = ros::Time::now();

		if ( !HULK.serial_isOpen() ) {
			ROS_FATAL("SERIAL COMMUNICATION FAILED");
			ros::shutdown();
			continue;
		}

		if ( !mpu_1.isConnected() ) {
			ROS_FATAL("MPU 1 COMMUNICATION FAILED");
			ros::shutdown();
			continue;
		}

		HULK.get_all();
		mpu_1.read_all();
		hulk_battery();
		hulk_odometry();

		data_axis accel = mpu_1.accel();
		data_axis gyro = mpu_1.gyro();

		if ( HULK.volt_bat_avg() < th_battery_min ) {
			ROS_FATAL("Battery level extremely low: Shutting down the node");
			ros::shutdown();
			continue;
		}

		hulk_position.header.stamp = current_time;
		hulk_imu.header.stamp = current_time;

		q.setRPY(0, 0, theta);
		geometry_msgs::Quaternion odom_quat = tf2::toMsg(q);
		hulk_position.pose.pose.position.x = x;
		hulk_position.pose.pose.position.y = y;
		hulk_position.pose.pose.position.z = 0.0;
		hulk_position.pose.pose.orientation = odom_quat;

		hulk_position.twist.twist.linear.x = trans_speed;
		hulk_position.twist.twist.linear.y = 0.0;
		hulk_position.twist.twist.angular.z = theta_speed;

		hulk_imu.angular_velocity.x = gyro.x;
		hulk_imu.angular_velocity.y = gyro.y;
		hulk_imu.angular_velocity.z = gyro.z;
		hulk_imu.linear_acceleration.x = accel.x;
		hulk_imu.linear_acceleration.y = accel.y;
		hulk_imu.linear_acceleration.z = accel.z;

		hulk_speed.x = HULK.left_speed();
		hulk_speed.y = HULK.right_speed();

		zero_vel_pub.publish(zero_cmd);
		info_odometry.publish(hulk_position);
		info_odometry_simplified.publish(odom);
		info_mpu.publish(hulk_imu);
		info_velocity.publish(hulk_speed);
		info_battery.publish(hulk_data);

		ros::spinOnce();

		freq.sleep();
	}

return 0;
}

void set_speed_rpm(const geometry_msgs::Twist::ConstPtr& speed){
	float v = speed->linear.x;
	float w = speed->angular.z;

	float right_speed_rad = (2*v+w*L)/(2*R);
	float left_speed_rad  = (2*v-w*L)/(2*R);

	float right_speed_rpm = (right_speed_rad*60/(2*PI));
	float left_speed_rpm = (left_speed_rad*60/(2*PI));

	HULK.set_speed(right_speed_rpm,left_speed_rpm);
}

void hulk_battery(){
	hulk_data.voltage = HULK.volt_bat();
	hulk_data.current = (HULK.current_right()*1000+HULK.current_left()*10);
	hulk_data.percentage = (HULK.volt_bat()/24)*100.0;
}

void hulk_odometry(){
	ros::Time current_time_odom = ros::Time::now();
	double dt = (current_time_odom - last_time_odom).toSec();

	float left_speed_rpm = HULK.left_speed();
	float right_speed_rpm = HULK.right_speed();

	float left_speed_rad = (left_speed_rpm*2*PI)/60;
	float right_speed_rad = (right_speed_rpm*2*PI)/60;

	float left_speed_m = left_speed_rad*R;
	float right_speed_m = right_speed_rad*R;

	theta_speed = (right_speed_m - left_speed_m)/L;
	trans_speed = (left_speed_m + right_speed_m)/2;

	theta = theta + theta_speed*dt;
	theta = atan2(sin(theta),cos(theta));

	x = x + trans_speed*cos(theta)*dt;
	y = y + trans_speed*sin(theta)*dt;

	last_time_odom = current_time_odom;
}	

void simplified_odometry(const nav_msgs::Odometry::ConstPtr& msg)
{
    tf2::Quaternion q_temp(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    tf2::Matrix3x3 m(q_temp);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

	odom.x = msg->pose.pose.position.x;
    odom.y = msg->pose.pose.position.y;
	odom.z = yaw;
}

void init_msgs() {
	hulk_position.pose.covariance[0] = 1e-2;
	hulk_position.pose.covariance[7] = 1e-2;
	hulk_position.pose.covariance[35] = 1e-2;
	hulk_position.twist.covariance[0] = 1e-2;
	hulk_position.twist.covariance[7] = 1e-2;
	hulk_position.twist.covariance[35] = 1e-2;

	hulk_position.pose.covariance[14] = 1e6;
	hulk_position.pose.covariance[21] = 1e6;
	hulk_position.pose.covariance[28] = 1e6;
	hulk_position.twist.covariance[14] = 1e6;
	hulk_position.twist.covariance[21] = 1e6;
	hulk_position.twist.covariance[28] = 1e6;

	hulk_imu.header.frame_id = "imu_link";
	
	hulk_imu.angular_velocity_covariance[0] = 1e-2;
	hulk_imu.angular_velocity_covariance[4] = 1e-2;
	hulk_imu.angular_velocity_covariance[8] = 1e-2;
	hulk_imu.linear_acceleration_covariance[0] = 1e-1;
	hulk_imu.linear_acceleration_covariance[4] = 1e-1;
	hulk_imu.linear_acceleration_covariance[8] = 1e-1;

	hulk_imu.orientation.x = 0.0;
	hulk_imu.orientation.y = 0.0;
	hulk_imu.orientation.z = 0.0;
	hulk_imu.orientation.w = 1.0;
	hulk_imu.orientation_covariance[0] = -1;
}