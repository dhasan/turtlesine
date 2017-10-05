
#include "turtlesine.h"
#include <sstream>
#include <geometry_msgs/Twist.h>
#include <turtlesim/TeleportAbsolute.h> 

const char TurtleSine::node_name[] = "turtlesine";

TurtleSine::TurtleSine()
{

	pubsine = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000); 
	clienttelep = n.serviceClient<turtlesim::TeleportAbsolute>("turtle1/teleport_absolute");
}

int TurtleSine::initialize(double x, double y, double theta)
{
	turtlesim::TeleportAbsolute telep;

	telep.request.x = x;
	telep.request.y = y;
	telep.request.theta = theta;

	while (!clienttelep.call(telep))
	{
		ROS_INFO("Turtel teleporting....");
		
	}
	return 0;
	
}

void TurtleSine::run(double lr, double amp) const // lr is loop rate
{
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
		std::cout << "Unable to teleport "<< std::endl;

	}else{
		ts->run(1.3, 2.0);
	}
	delete ts;
}
