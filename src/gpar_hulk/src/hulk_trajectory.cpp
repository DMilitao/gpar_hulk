#include "ros/ros.h"
#include <sstream>
#include "serial/serial.h"
#include "geometry_msgs/Twist.h"
#include <vector>
#include "math.h"
#include "geometry_msgs/Point.h"

void position_control_law(const geometry_msgs::Point::ConstPtr& position);

float x, y, theta;

float angular;
float linear;

int aux_ref = 0;
std::vector<float> x_ref = {0, 0, 0, 0};
std::vector<float> y_ref = {0, 0, 0, 0};

float tol_ref = 0.05;
float tol_ang = 0.05;
float error_x = 1;
float error_y = 1;
float error_theta = 0;

int main(int argc, char **argv)
{
	x_ref.at(1) = std::stof(argv[1]);
	x_ref.at(2) = std::stof(argv[1]);
	y_ref.at(2) = std::stof(argv[1]);
	y_ref.at(3) = std::stof(argv[1]);

	ros::init(argc, argv, "hulk_trajectory");
	ros::NodeHandle n;
	
	geometry_msgs::Twist speed;
	ros::Publisher set_speed = n.advertise<geometry_msgs::Twist>("speed", 1000);
	ros::Subscriber get_position = n.subscribe("/hulk_node/odometry_filtered_simplifed", 1000, position_control_law);

	ros::Rate freq(20);
	while(ros::ok()){

		speed.linear.x = linear;
		speed.angular.z = angular;

		set_speed.publish(speed);
		
		ros::spinOnce();		
		freq.sleep();
	}

return 0;
}

void position_control_law(const geometry_msgs::Point::ConstPtr& position){

	
	error_x = x_ref[aux_ref] - position->x;
	error_y = y_ref[aux_ref] - position->y;
	error_theta = std::atan2(error_y, error_x) - position->z;
	error_theta = atan2(sin(error_theta),cos(error_theta));

	angular = 0.8*angular + 0.5*error_theta;
	angular = fmax(fmin(angular, 0.6), -0.6);

	if ( std::abs(error_theta) < tol_ang ) {
		linear = 4*sqrt(error_x*error_x + error_y*error_y);
		linear = fmax(fmin(linear, 0.2), -0.2);
	} else {
		linear = linear/1.1;
	}

	std::stringstream ss;
	ss << "Odometry: X " << position->x << " | Y " << position->y << " | Theta " << position->z << " | Linear: " << linear << " | Angular: " << angular << " | Objective: " << x_ref[aux_ref] << y_ref[aux_ref] ;
	ROS_INFO(ss.str().c_str());

	if ((std::abs(error_x) < tol_ref) && (std::abs(error_y) < tol_ref)) {
		aux_ref++;
	}

	aux_ref = aux_ref > 3 ? 0 : aux_ref;
}