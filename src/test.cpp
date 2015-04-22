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


	

    class Spinner {
      ros::NodeHandle n_;
      ros::Publisher spin_pub;
      ros::Subscriber sub;
    public:
      Spinner(ros::NodeHandle n) :
        n_(n)
      {
        spin_pub = n.advertise<geometry_msgs::Twist> ("/turtlebot_node/cmd_vel", 1000);
        sub = n.subscribe("/robot_pose_ekf/odom_combined", 1000,  spin_cb);  //Does not work!
      }
     
      void spin_cb(const geometry_msgs::PoseWithCovarianceStamped msg)
      {
        //do stuff
      }
    };
     
    int main(int argc, char** argv)
    {
      ros::init(argc, argv, "spin");
      ros::NodeHandle n;
      Spinner spin_object = Spinner(n);
      ros::spin();
      return 0;
    }


