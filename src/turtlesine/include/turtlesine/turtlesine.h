
#ifndef _TURTLESINE_H_
#define _TURTLESINE_H_

#include "ros/ros.h"


class TurtleSine 
{

private:
	ros::NodeHandle n;
	ros::Publisher pubsine;
	ros::ServiceClient clienttelep;

public:
	static const std::string node_name;
	TurtleSine();

	int initialize(double x, double y, double theta);
	void run(double lr, double amp) const;


	
};

#endif