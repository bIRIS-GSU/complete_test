//Movement demo

//Ros Header
#include "ros/ros.h"

//Msg headers
#include <std_msgs/String.h>

#include <geometry_msgs/Twist.h>

#include <sensor_msgs/Image.h> //Depth Image Msgs
#include <sensor_msgs/CameraInfo.h> //CameraInfo Msgs
#include <sensor_msgs/LaserScan.h> //LaserScan Msgs

#include <sstream>

ros::Publisher chatter_pub;



void LsrInfo(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	int times_run;
	
	//ros::param::set("/camera/depthimage_to_laserscan_loader/range_max", 10.0);
	ROS_INFO("Begin loop - LaserScan Info");
	ROS_INFO("Range Min: [%f]", msg->range_min);
  	ROS_INFO("Range Max: [%f]", msg->range_max);
  	ROS_INFO("Size of Ranges: [%lu]",msg->ranges.size());
  	float minVal = (msg->range_max); //to begin loop
  	float finalVal = 0;
  	for (int i = 0;i<(msg->ranges.size()); i++){
  		if ((msg->range_min) <= (msg->ranges[i]) && (msg->ranges[i]) <= (msg->range_max)){  //if in range, valid dist.
  			if ((msg->ranges[i]) < minVal){													//if lower than prev. min. dist.
  				minVal = (msg->ranges[i]);
  				finalVal = minVal;
  			}
  		}
  	}
  	
  	
  	ROS_INFO("Min. Dist: [%f]", finalVal);
  	
	ROS_INFO("End loop - LaserScan Info");
	times_run++;
	ROS_INFO("---Times Run--- :[%i]", times_run);
	
	geometry_msgs::Twist vel;
	vel.linear.x = 0.1;
  	vel.angular.z = 0;
  	ROS_INFO("%f and %f", vel.linear.x, vel.angular.z);
  	
  	// %Tag(PUBLISH)%
   
   //PubClass run_now;
   chatter_pub.publish(vel);
	// %EndTag(PUBLISH)%
	
	
}




int main(int argc, char **argv)
{

	// %Tag(INIT)%
	ros::init(argc, argv, "move_test");
	// %EndTag(INIT)%
	
	// %Tag(NODEHANDLE)%
  	ros::NodeHandle n;
	// %EndTag(NODEHANDLE)%
	
	ros::Subscriber sub3 = n.subscribe("/scan", 0, LsrInfo);
	chatter_pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
	ros::spinOnce();
	return 0;
	
}
