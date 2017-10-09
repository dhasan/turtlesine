
#include "turtlesine.h"
#include <sstream>
#include <geometry_msgs/Twist.h>
#include <turtlesim/TeleportAbsolute.h> 

const std::string TurtleSine::node_name = "turtlesine";

TurtleSine::TurtleSine() : n("~"), pubsine(n.advertise<geometry_msgs::Twist>("cmd_vel", 1000)),
	clienttelep(n.serviceClient<turtlesim::TeleportAbsolute>("teleport_absolute")), 
	timer(n.createTimer(ros::Duration(1.0/1.3), boost::bind(&TurtleSine::timerCallback, const_cast<TurtleSine*>(this), 4.44, 4.44))),
	lastpose(3){}


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
		lastpose.at(POSE_X) = telep.request.x;  //TODO: better to use response values if available
		lastpose.at(POSE_Y) = telep.request.y;
		lastpose.at(POSE_THETA) = telep.request.theta;
		return 0;
	}
	else{
		ROS_ERROR("Unable to teleport the turtle.");
		return -1;
	}
	
}

void TurtleSine::timerCallback(TurtleSine *obj, double l, double a)
{
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

  	obj->poseCalculate(twist);
  
	++count;
}

void TurtleSine::poseCalculate(const geometry_msgs::Twist &twist){

	/* Odometry formula*/

	double dt = 1.0/1.3; //time discrete
	double vx = twist.linear.x;
	double vy = twist.linear.y; 
	double th = twist.angular.z;

	double thi = lastpose.at(POSE_THETA); //initial angle

	double delta_x = (vy * sin(thi) - vx * cos(thi)) * dt;
	double delta_y = (vy * cos(thi) + vx * sin(thi)) * dt;
	double delta_th = th * dt;

	lastpose.at(POSE_X) += (float)delta_x;
	lastpose.at(POSE_Y) += (float)delta_y;
	lastpose.at(POSE_THETA) += (float)delta_th;

	ROS_INFO("Calculated pose x y: %f %f", lastpose.at(0), lastpose.at(1));
	//TODO: wall hit not taken into account
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, TurtleSine::node_name);

	TurtleSine ts;

	if (ts.initialize()){

		std::cout << "Unable to teleport. Turtlesim_naode might be missing"<< std::endl;
		exit(0);

	}

	ros::spin();

	return 0;
}
