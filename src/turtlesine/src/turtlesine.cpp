
#include "turtlesine.h"
#include <sstream>
#include <geometry_msgs/Twist.h>
#include <turtlesim/TeleportAbsolute.h>
#include <turtlesim/Spawn.h>
#include <tf/transform_datatypes.h>
#include "turtlesine/Odom.h"
#include <pluginlib/class_list_macros.h>
#include "nodelet/loader.h"
#include <turtlesim/Kill.h>


PLUGINLIB_EXPORT_CLASS(task1_pkg::TurtleSine, nodelet::Nodelet)

namespace task1_pkg {

	const std::string TurtleSine::node_name = "turtlesine";

	

	TurtleSine::TurtleSine() : nh(getNodeHandle()){}
	
	TurtleSine::TurtleSine(ros::NodeHandle &n) : nh(n), pubsine(nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000)),
		clienttelep(nh.serviceClient<turtlesim::TeleportAbsolute>("teleport_absolute")), 
		swan(nh.serviceClient<turtlesim::Spawn>("/task1/sim/spawn")),
		timer(nh.createTimer(ros::Duration(TIME_DT), boost::bind(&TurtleSine::timerCallback, const_cast<TurtleSine*>(this), 4.44, 4.44))),
		odompub(nh.advertise<turtlesine::Odom>("odompub", 1000)),
		lastpose(3)
	{
		turtlesim::TeleportAbsolute telep;
		
	
		int cnt = 10; //retrys
		
		
		/*
			Apparently params can't be float, only (str|int|double|bool|yaml), so use temporary vars, the other option is yaml with single vector parameter...
		*/
		double tx,ty,ttheta;
		nh.getParam("turtlesine/initial_x", tx);
		nh.getParam("turtlesine/initial_y", ty);
		nh.getParam("turtlesine/initial_theta", ttheta);
	
		std::cout << "NAMESPACE : " <<ros::this_node::getNamespace()<<std::endl;
	
		telep.request.x = tx;
		telep.request.y = ty;
		telep.request.theta = ttheta;
	
		/*
			Since both nodes are starting at the same from launcher sometimes turtlesine node starts before
			turtlesim_node, so we need to wait until turtlesim_node appear to use teleport service
		*/
	
		while (!ros::service::exists("teleport_absolute", true) && cnt)
		{
			
			ROS_WARN("Wait for turtlesim_node and Teleport service server..");
			ros::service::waitForService("teleport_absolute", 50);
			--cnt;
		}
	
		if (cnt){
			ROS_INFO("Turtle teleported.");
			clienttelep.call(telep);
			
		}
		else{
			ROS_ERROR("Unable to teleport the turtle.");
			exit(0);
			
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
		turtlesine::Odom odom;
	
		double dt = TIME_DT; //time discrete
		double vx = twist.linear.x;
		double vy = twist.linear.y; 
		double th = twist.angular.z;
	
		double thi = lastpose.at(POSE_THETA); //initial angle
	
		double delta_x = (vx * cos(thi) - vy * sin(thi)) * dt;
		double delta_y = (vx * sin(thi) + vy * cos(thi)) * dt;
		double delta_th = th * dt;
	
		lastpose.at(POSE_X) += (float)delta_x;
		lastpose.at(POSE_Y) += (float)delta_y;
		lastpose.at(POSE_THETA) += (float)delta_th;
	
		ROS_INFO("Calculated pose x y: %f %f", lastpose.at(0), lastpose.at(1));
	
		//q.setEuler(lastpose.at(POSE_THETA), 0, 0);
		tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, lastpose.at(POSE_THETA));
		odom.q.x = q[0];
		odom.q.y = q[1];
		odom.q.z = q[2];
		odom.q.w = q[3];
	
		odom.header.stamp = ros::Time::now();
	
		odom.p.x = lastpose.at(POSE_X);
		odom.p.y = lastpose.at(POSE_Y);
		odom.p.z = 0;
	
		odompub.publish(odom);
	
		
	}
	
	void TurtleSine::onInit()
	{
		NODELET_DEBUG("Initializing nodelet...");
		pubsine = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
		clienttelep = nh.serviceClient<turtlesim::TeleportAbsolute>("teleport_absolute"); 
		swan = nh.serviceClient<turtlesim::Spawn>("spawn"); 
		timer = nh.createTimer(ros::Duration(TIME_DT), boost::bind(&TurtleSine::timerCallback, const_cast<TurtleSine*>(this), 4.44, 4.44));
		odompub = nh.advertise<turtlesine::Odom>("odompub", 1000);
		lastpose.resize(3);
	}

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, task1_pkg::TurtleSine::node_name);
#if 1
	
	ros::NodeHandle n("/");
	task1_pkg::TurtleSine ts(n);

#else
	nodelet::Loader nodelet;
  	nodelet::M_string remap(ros::names::getRemappings());
  	nodelet::V_string nargv;
  	std::string nodelet_name = ros::this_node::getName();
  	nodelet.load(nodelet_name, "task1_pkg/TurtleSine", remap, nargv);
#endif
	ros::spin();

	return 0;
}
