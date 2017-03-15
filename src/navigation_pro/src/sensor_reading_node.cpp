/**
 * \file goal_generator_node.cpp
 * \A node to control a slave turtle to follow another turtle
 * \author Gaetan Garcia
 * \version 0.1
 * \date 26 April 2016
 * 
 * \param[in] "Kpos": pose parameter, default value, 0.5 .
 * \param[in] "Kang": angular parameter, default value, 4.0.
 * 
 * Subscribes to: 
 *    ° Relative topic "leader_pose" and "follower_pose"
 * 
 * Publishes to: 
 *    ° Relative topic "follower_velocity"
 *
 *
 */

#include <vector>
#include <string>
#include <map>
#include <list>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

//ROS
#include "ros/ros.h"

//ROS msgs
#include <sensor_msgs/LaserScan.h>


//Namespaces
using namespace std;

//Global variables
ros::Publisher pub_command ;

        
void sensorCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
    ROS_INFO("Sensor data restructured");
    
    sensor_msgs::LaserScan new_scan;
    new_scan.header.seq = 1;
    new_scan.header.stamp = ros::Time::now();
    new_scan.header.frame_id = msg->header.frame_id;
    new_scan.angle_min = msg->angle_min;
    new_scan.angle_max = msg->angle_max;
    new_scan.angle_increment = msg->angle_increment;
    new_scan.scan_time = msg->scan_time;
    new_scan.range_min = msg->range_min;
    new_scan.range_max = msg->range_max;
          
    
    for (unsigned int i=0;i<msg->ranges.size();i++) {
		if(isnan(msg->ranges[i])){
			new_scan.ranges.push_back(msg->range_max+2.0);
		}else{
			new_scan.ranges.push_back(msg->ranges[i]);
		}
	}
    pub_command.publish(new_scan);
}


int main (int argc, char** argv)
{

    //ROS Initialization
    ros::init(argc, argv, "sensor_reading_node");
    ROS_INFO("Connected !!!!!");
    ros::NodeHandle nh_("~");//ROS Handler - local namespace.
	

    //Subscribing
    ROS_INFO("Subscribing to topics\n");

    ros::Subscriber Map
        = nh_.subscribe<sensor_msgs::LaserScan> ("/scan"  , 1, sensorCallback);
	
    //Publishing follower velocity
    pub_command = nh_.advertise<sensor_msgs::LaserScan>("/scan_revised", 1);

    ros::Rate rate(10) ;
    ROS_INFO("Goal node spinning @ 40Hz");
	
    while (ros::ok()){
		
        // Attend callbacks
        ros::spinOnce();
		
		

        rate.sleep();
    } 

    ROS_INFO("ROS-Node Terminated\n");
}




