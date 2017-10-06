
#include "turtlesine.h"
#include <sstream>
#include <geometry_msgs/Twist.h>
#include <turtlesim/TeleportAbsolute.h> 

const std::string TurtleSine::node_name = "turtlesine";

TurtleSine::TurtleSine() : n("~"), pubsine(n.advertise<geometry_msgs::Twist>("cmd_vel", 1000)),
	clienttelep(n.serviceClient<turtlesim::TeleportAbsolute>("teleport_absolute"))
{

	//pubsine = n.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000); 
	//clienttelep = n.serviceClient<turtlesim::TeleportAbsolute>("turtle1/teleport_absolute");
}
TurtleSine::~TurtleSine(){}
int TurtleSine::initialize(/*double x, double y, double theta*/)
{
	turtlesim::TeleportAbsolute telep;
	int cnt = 500; //retrys
	
	/*
		Apparently params can't be float only (str|int|double|bool|yaml), so use temporary vars, the other option is yaml with single vector parameter...
	*/
	double tx,ty,ttheta;
	n.getParam("initial_x", tx);
	n.getParam("initial_y", ty);
	n.getParam("initial_theta", ttheta);


	telep.request.x = (float)tx;
	telep.request.y = (float)ty;
	telep.request.theta = (float)ttheta;

	ROS_INFO("telep.request.y %f",ty);
	//telep.request.x = ;
	//telep.request.y = vec.at(1);
	//telep.request.theta = vec.at(2);

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

	TurtleSine ts;

	if (ts.initialize(/*0.0, 5.55, 1.5*/)){
		//delete ts;
		std::cout << "Unable to teleport. Turtlesim_naode might be missing"<< std::endl;
		exit(0);

	}else{
		ts.run(1.3, 2.0);
	}

	//delete ts;
	return 0;
}
