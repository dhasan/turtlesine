
#ifndef _TURTLESINE_H_
#define _TURTLESINE_H_

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

class TurtleSine 
{

private:
	

	ros::NodeHandle n;
	ros::Publisher pub;

public:
	static const char node_name[];
	TurtleSine(double x, double y, double alpha);
	void publish(double lr);


	
};

#endif