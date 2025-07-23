//Here's only a structure to a future service to HULK
#include "ros/ros.h"
#include "gpar_hulk/test.h"
#include <string>

bool function(gpar_hulk::test::Request &req, gpar_hulk::test::Response &res){
	res.msg = "hello";	
	return true;
}


int main(int argc, char** argv){
	ros::init(argc,argv,"server");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("test",function);
	ros::spin();

	return 0;
}


