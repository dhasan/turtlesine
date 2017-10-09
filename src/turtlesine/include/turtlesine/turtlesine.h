
#ifndef _TURTLESINE_H_
#define _TURTLESINE_H_

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

#define		POSE_X		(0)
#define		POSE_Y		(1)
#define		POSE_THETA	(2)

class TurtleSine 
{

private:
	ros::NodeHandle n;


	ros::Publisher pubsine;
	ros::ServiceClient clienttelep;
	ros::Timer timer;

	std::vector<float> lastpose;

	static void timerCallback(TurtleSine *obj,double l, double a);
	void poseCalculate(const geometry_msgs::Twist &twist);

public:
	static const std::string node_name;
	TurtleSine();
	~TurtleSine() = default;

	int initialize();
	//void run(double lr, double amp) const;


	
};

#endif