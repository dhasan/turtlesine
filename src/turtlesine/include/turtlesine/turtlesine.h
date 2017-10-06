
#ifndef _TURTLESINE_H_
#define _TURTLESINE_H_

#include "ros/ros.h"


class TurtleSine 
{

private:
	ros::NodeHandle n;

	ros::Publisher pubsine;
	ros::ServiceClient clienttelep;

	static void timerCallback(const TurtleSine *obj,double l, double a);

public:
	static const std::string node_name;
	TurtleSine();
	~TurtleSine();

	int initialize();
	void run(double lr, double amp) const;


	
};

#endif