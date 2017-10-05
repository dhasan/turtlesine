
#ifndef _TURTLESINE_H_
#define _TURTLESINE_H_

#include "ros/ros.h"
//#include "std_msgs/String.h"

//#include <sstream>

class TurtleSine 
{

private:
	ros::NodeHandle n;
	ros::Publisher pubsine;
	ros::ServiceClient clienttelep;
	//ros::Publisher pubtelep;

public:
	static const char node_name[];
	TurtleSine();

	int initialize(double x, double y, double theta);
	void run(double lr, double amp) const;


	
};

#endif