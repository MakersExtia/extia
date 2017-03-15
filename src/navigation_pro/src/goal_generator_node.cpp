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


//ROS
#include "ros/ros.h"

//ROS msgs
#include <geometry_msgs/PoseStamped.h>

#include <move_base_msgs/MoveBaseActionGoal.h>


//Namespaces
using namespace std;

//Global variables
ros::Publisher pub_command ;



int main (int argc, char** argv)
{

    //ROS Initialization
    ros::init(argc, argv, "goal_generator_node");
    ROS_INFO("Connected !!!!!");
    ros::NodeHandle nh_("~");//ROS Handler - local namespace.
	
	double goal_px;
	double goal_py;
	double goal_oz;
	double goal_ow;
	
	nh_.param("goal_px",goal_px,0.0);
	nh_.param("goal_py",goal_py,0.0);
	
	nh_.param("goal_oz",goal_oz,0.0);
	nh_.param("goal_ow",goal_ow,1.0);	
	
    //Subscribing
    ROS_INFO("Subscribing to topics\n");

    //Publishing follower velocity
    pub_command = nh_.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1);

    ros::Rate rate(5) ;
    ROS_INFO("Goal node spinning @ 5Hz");
	
    while (ros::ok()){
		
        // Attend callbacks
        ros::spinOnce();
				
		geometry_msgs::PoseStamped pose_stamp;
		pose_stamp.header.seq = 1;
		pose_stamp.header.stamp = ros::Time::now();
		pose_stamp.header.frame_id = "/map";
		pose_stamp.pose.position.x = goal_px;
		pose_stamp.pose.position.y = goal_py;
		pose_stamp.pose.position.z = 0.0;
		pose_stamp.pose.orientation.x = 0.0;
		pose_stamp.pose.orientation.y = 0.0;
		pose_stamp.pose.orientation.z = goal_oz;
		pose_stamp.pose.orientation.w = goal_ow;
 
		move_base_msgs::MoveBaseActionGoal cmd_goal;
		cmd_goal.header.seq = 1;
		cmd_goal.header.stamp = ros::Time::now();
		cmd_goal.header.frame_id = "/map";
		cmd_goal.goal_id.stamp = ros::Time::now();
		cmd_goal.goal_id.id = "goal1";
		cmd_goal.goal.target_pose = pose_stamp;
		
		pub_command.publish(cmd_goal);

        rate.sleep();
    } 

    ROS_INFO("ROS-Node Terminated\n");
}




