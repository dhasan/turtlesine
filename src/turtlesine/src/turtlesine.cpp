
#include "turtlesine.h"
#include <sstream>
#include <geometry_msgs/Twist.h>
#include <turtlesim/TeleportAbsolute.h> 

const std::string TurtleSine::node_name = "turtlesine";

TurtleSine::TurtleSine()
{

	pubsine = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000); 
	clienttelep = n.serviceClient<turtlesim::TeleportAbsolute>("turtle1/teleport_absolute");
}

int TurtleSine::initialize(double x, double y, double theta)
{
	turtlesim::TeleportAbsolute telep;
	int cnt = 500; //retrys

	telep.request.x = x;
	telep.request.y = y;
	telep.request.theta = theta;

	/*
		Since both nodes are starting at the same from launcher time sometimes turtlesine node starts before
		turtlesim_node, so we need to wait until turtlesim_node appear to use teleport service
	*/
	while (!clienttelep.call(telep) && cnt)
	{
		ROS_WARN("Wait for turtlesim_node and Teleport service server..");
		cnt--;
	}
	if (cnt){
		ROS_INFO("Turtle teleported.");
		return 0;
	}
	else{
		ROS_ERROR("Unable to teleport the turtle.");
		return -1;
	}
	
}

void TurtleSine::run(double lr, double amp) const // lr is loop rate
{
	/*
		This implementation is without feedback from pose topic, it relies on the duration on twist msgs / loop rate and velocity
	*/
	ros::Rate loop_rate(lr);
	geometry_msgs::Twist twist;
	int count = 0;
	
	twist.linear.x = amp;
	twist.linear.y = 0;
	twist.linear.z = 0;

	twist.angular.x = 0;
	twist.angular.y = 0;
	twist.angular.z = amp;

	while (ros::ok())
  	{

  		if (count & 1){
  			twist.angular.z *= -1;
  		}

  		pubsine.publish(twist);
  		ros::spinOnce();

    	loop_rate.sleep();


  		++count;
  	}

}



int main(int argc, char **argv)
{
	ros::init(argc, argv, TurtleSine::node_name);

	TurtleSine *ts = new TurtleSine();

	if (ts->initialize(0.0, 5.8, 1.5)){
		delete ts;
		std::cout << "Unable to teleport. Turtlesim_naode might be missing"<< std::endl;
		exit(0);

	}else{
		ts->run(1.3, 2.0);
	}

	delete ts;
	return 0;
}
