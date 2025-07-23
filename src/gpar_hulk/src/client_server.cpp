//You can use this CLIENT-SERVER to create a service more complex
#include "ros/ros.h"
#include "gpar_hulk/test.h"
#include <cstdlib>

int main(int argc, char** argv){
	ros::init(argc,argv,"client_server");

	ros::NodeHandle n;

	ros::ServiceClient client = n.serviceClient<gpar_hulk::test>("test");

	gpar_hulk::test srv;

	srv.request.a = atoll(argv[1]);

	if(client.call(srv)){
		ROS_INFO("Answer = %s",srv.response.msg.c_str());
	}
	else{
		ROS_ERROR("Failed to call the service");
		return 1;
	}

	return 0;
}
