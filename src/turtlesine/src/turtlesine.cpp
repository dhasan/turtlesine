
#include "turtlesine.h"
#include <sstream>

//TurtleSine::node_name = "turtlesine";
const char TurtleSine::node_name[] = "this way it works";

TurtleSine::TurtleSine(double x, double y, double alpha)
{

	//pub = n.advertise<geometry_msgs::Twist>("chatter", 1000);
}

void TurtleSine::publish(double lr) 
{

}



int main(int argc, char **argv)
{
	ros::init(argc, argv, TurtleSine::node_name);

	TurtleSine *ts = new TurtleSine(0, 40, 0.5);

	ts->publish(1);
	
	delete ts;
}
