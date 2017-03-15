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

#include "dynamicvoronoi.h"

//ROS
#include "ros/ros.h"

//ROS msgs
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>

#include <move_base_msgs/MoveBaseActionGoal.h>

#define FREE 0xFF
#define UNKNOWN 0x80
#define OCCUPIED 0x00
#define WIN_SIZE 800

//Namespaces
using namespace std;

//Global variables

bool ready=false;
        
        
void mapReceivedCallback(const nav_msgs::OccupancyGridConstPtr& msg){
	
    ROS_INFO("Map received");
    cout << msg->info.width << endl;
    cout << msg->info.height << endl;
    
    nav_msgs::MapMetaData info_ = msg->info;
     std::string frame_id_ = msg->header.frame_id;
     
     
     bool **map=NULL;
     map = new bool*[msg->info.width];

	for (int x=0; x<msg->info.width; x++) {
		map[x] = new bool[msg->info.height];
	}
     
     cv::Mat_<uint8_t> og_, cropped_og_;
        cv::Mat_<cv::Vec3b> og_rgb_, og_rgb_marked_;
        cv::Point3i og_center_;
            // Create an image to store the value of the grid.
            og_ = cv::Mat_<uint8_t>(msg->info.height, msg->info.width,0xFF);
            og_center_ = cv::Point3i(-info_.origin.position.x/info_.resolution,
                    -info_.origin.position.y/info_.resolution,0);

            // Some variables to select the useful bounding box 
            unsigned int maxx=0, minx=msg->info.width, 
                         maxy=0, miny=msg->info.height;
            // Convert the representation into something easy to display.
            for (unsigned int j=0;j<msg->info.height;j++) {
                for (unsigned int i=0;i<msg->info.width;i++) {
                    int8_t v = msg->data[j*msg->info.width + i];
                    switch (v) {
                        case 0: 
                            og_(j,i) = FREE; 
                            map[i][j]= false;
                            break;
                        case 100: 
                            og_(j,i) = OCCUPIED; 
                            map[i][j]= true;
                            break;
                        case -1: 
                        default:
                            og_(j,i) = UNKNOWN; 
                            map[i][j]= false;
                            break;
                    }
                    // Update the bounding box of free or occupied cells.
                    if (og_(j,i) != UNKNOWN) {
                        minx = std::min(minx,i);
                        miny = std::min(miny,j);
                        maxx = std::max(maxx,i);
                        maxy = std::max(maxy,j);
                    }
                }
            }
            // dilatation/erosion part
            int erosion_type = cv::MORPH_RECT ;
            int erosion_size = 0.05/info_.resolution ;
            cv::Mat element = cv::getStructuringElement(erosion_type,
                    cv::Size(2*erosion_size+1,2*erosion_size+1),
                    cv::Point( erosion_size, erosion_size));
            cv::erode( og_, og_, element );
            // -----------------------
            if (!ready) {
                ready = true;
                ROS_INFO("Received occupancy grid, ready to plan");
            }
            cv::cvtColor(og_, og_rgb_, CV_GRAY2RGB);
            /*// The lines below are only for display
            unsigned int w = maxx - minx;
            unsigned int h = maxy - miny;
            cv::Rect roi_ = cv::Rect(minx,miny,w,h);
            
            // Compute a sub-image that covers only the useful part of the
            // grid.
            cropped_og_ = cv::Mat_<uint8_t>(og_,roi_);
            if ((w > WIN_SIZE) || (h > WIN_SIZE)) {
                // The occupancy grid is too large to display. We need to scale
                // it first.
                
                double ratio = w / ((double)h);
                cv::Size new_size;
                if (ratio >= 1) {
                    new_size = cv::Size(WIN_SIZE,WIN_SIZE/ratio);
                } else {
                    new_size = cv::Size(WIN_SIZE*ratio,WIN_SIZE);
                }
                cv::Mat_<uint8_t> resized_og;
                cv::resize(cropped_og_,resized_og,new_size);
                cv::imshow( "OccGrid", resized_og );
            } else {
                // cv::imshow( "OccGrid", cropped_og_ );
                cv::imshow( "OccGrid", og_rgb_ );
            }*/
            cv::imshow( "OccGrid", og_rgb_ );
            
            DynamicVoronoi voronoi;
			voronoi.initializeMap(msg->info.width, msg->info.height, map);
			voronoi.update(); // update distance map and Voronoi diagram
			voronoi.prune();  // prune the Voronoi
			

			voronoi.visualize("voronoi.ppm");
            
            if (cv::waitKey( 50 )== 'q') {
            ros::shutdown();
            }
            
            
            ROS_INFO("Finished callback");
}


int main (int argc, char** argv)
{

    //ROS Initialization
    ros::init(argc, argv, "show_map_node");
    ROS_INFO("Connected !!!!!");
    ros::NodeHandle nh_("~");//ROS Handler - local namespace.
	

    //Subscribing
    ROS_INFO("Subscribing to topics\n");

    ros::Subscriber Map
        = nh_.subscribe<nav_msgs::OccupancyGrid> ("/map"  , 1, mapReceivedCallback);

    ros::Rate rate(20) ;
    ROS_INFO("Goal node spinning @ 20Hz");
	cv::namedWindow( "OccGrid", CV_WINDOW_AUTOSIZE );
	//cv::namedWindow( "VoronoiGrid", CV_WINDOW_AUTOSIZE );
	
    while (ros::ok()){
		
		if (cv::waitKey( 50 )== 'q') {
            ros::shutdown();
        }
		
        // Attend callbacks
        ros::spinOnce();
		

        rate.sleep();
    } 

    ROS_INFO("ROS-Node Terminated\n");
}




