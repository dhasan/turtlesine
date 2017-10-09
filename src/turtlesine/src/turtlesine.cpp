
#include "turtlesine.h"
#include <sstream>
#include <geometry_msgs/Twist.h>
#include <turtlesim/TeleportAbsolute.h> 

const std::string TurtleSine::node_name = "turtlesine";

TurtleSine::TurtleSine() : n("~"), pubsine(n.advertise<geometry_msgs::Twist>("cmd_vel", 1000)),
	clienttelep(n.serviceClient<turtlesim::TeleportAbsolute>("teleport_absolute")), 
	timer(n.createTimer(ros::Duration(1/1.3), boost::bind(&TurtleSine::timerCallback, const_cast<TurtleSine*>(this), 4.44, 4.44))){}

TurtleSine::~TurtleSine(){}
int TurtleSine::initialize()
{
	turtlesim::TeleportAbsolute telep;
	int cnt = 500; //retrys
	
	/*
		Apparently params can't be float, only (str|int|double|bool|yaml), so use temporary vars, the other option is yaml with single vector parameter...
	*/
	double tx,ty,ttheta;
	n.getParam("initial_x", tx);
	n.getParam("initial_y", ty);
	n.getParam("initial_theta", ttheta);


	telep.request.x = (float)tx;
	telep.request.y = (float)ty;
	telep.request.theta = (float)ttheta;

	/*
		Since both nodes are starting at the same from launcher sometimes turtlesine node starts before
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

void TurtleSine::timerCallback(const TurtleSine *obj, double l, double a)
{
	ROS_WARN("Check ptr2 %p %f %f", obj,l,a);
	
	//ros::Rate loop_rate(0.707);
	geometry_msgs::Twist twist;
	static int count = 0;
	twist.linear.x = l;
	twist.linear.y = 0;
	twist.linear.z = 0;

	twist.angular.x = 0;
	twist.angular.y = 0;
	twist.angular.z = a;

	if (!(count & 1)){
  		twist.angular.z *= -1;
  	}

  	obj->pubsine.publish(twist);
  
	++count;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, TurtleSine::node_name);

	TurtleSine ts;

	if (ts.initialize()){
		//delete ts;
		std::cout << "Unable to teleport. Turtlesim_naode might be missing"<< std::endl;
		exit(0);

	}

	ros::spin();
	

	//delete ts;
	return 0;
}
