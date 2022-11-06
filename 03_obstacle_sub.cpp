/**
 *  03 Obstacle Avoidance
 *  @file 03_obstacle_sub.cpp
 *  @version 1.0 10.04.2022
 *  @brief Simulation of a drone with obstacle avoidance
 * 
 *  @author Bernhard Leschinger
 *  Contact: bernhard.leschinger@student.uibk.ac.at
 *
 *  Drone will be assigned a linear waypoint in x direction
 *  and sent to that
 *  LIDAR is controlled the whole time by a callback function
 *  if an obstacle is in the range of 3 meters, avoidance starts
 *  subscribed to the ROS LIDAR messages and data
 *  @param
 *  none
 *  @return -
 *  example: 03_obstacle_sub 15 5
 */

#include <ros/ros.h>
#include <gnc_functions.hpp>
#include <sensor_msgs/LaserScan.h>

int mode_a = 0;
float mode_h = 2.0;

void scan_cb(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	sensor_msgs::LaserScan current_2D_scan;
  	current_2D_scan = *msg;
	float avoidance_vector_x = 0; 
	float avoidance_vector_y = 0;
	bool avoid = false;
	mode_a=0;

	//ROS_INFO("ARRAY SIZE is %lu ", current_2D_scan.ranges.size());
	for(int i=1; i<current_2D_scan.ranges.size(); i++)
	{
		float d0 = 3; 
		float k = 0.5;

		if(current_2D_scan.ranges[i] < d0 && current_2D_scan.ranges[i] > .35)
		{
			avoid = true;
			mode_a=1;
			float x = cos(current_2D_scan.angle_increment*i);
			float y = sin(current_2D_scan.angle_increment*i);
			float U = -.5*k*pow(((1/current_2D_scan.ranges[i]) - (1/d0)), 2);	

			avoidance_vector_x = avoidance_vector_x + x*U;
			avoidance_vector_y = avoidance_vector_y + y*U;

		}
	}
	float current_heading = get_current_heading();
	float deg2rad = (M_PI/180);
	avoidance_vector_x = avoidance_vector_x*cos((current_heading)*deg2rad) - avoidance_vector_y*sin((current_heading)*deg2rad);
	avoidance_vector_y = avoidance_vector_x*sin((current_heading)*deg2rad) + avoidance_vector_y*cos((current_heading)*deg2rad);

	if(avoid)
	{
		ROS_INFO("AVOID!");
		if( sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)) > 3)
		{
			avoidance_vector_x = 3 * (avoidance_vector_x/sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)));
			avoidance_vector_y = 3 * (avoidance_vector_y/sqrt(pow(avoidance_vector_x,2) + pow(avoidance_vector_y,2)));
		}
		geometry_msgs::Point current_pos;
		current_pos = get_current_location();
		//set_destination(current_pos.x + avoidance_vector_x, current_pos.y + avoidance_vector_y, 2, 0);
		mode_h = mode_h + 0.2;
		set_destination(current_pos.x + avoidance_vector_x, current_pos.y + avoidance_vector_y, mode_h, 0);
		// x,y -> -y,x orthogonal vector
		//set_destination(current_pos.x + avoidance_vector_x - 2*avoidance_vector_y, current_pos.y + avoidance_vector_y + 2*avoidance_vector_x, 2, 0);
	} else mode_h=2.0;

}


int main(int argc, char **argv)
{
	//initialize ros 
	ros::init(argc, argv, "avoidance_node");
	ros::NodeHandle n;
	
	ros::Subscriber collision_sub = n.subscribe<sensor_msgs::LaserScan>("/spur/laser/scan", 1, scan_cb);

	//initialize control publisher/subscribers
	init_publisher_subscriber(n);

	// wait for FCU connection
	wait4connect();

	//wait for user to switch to mode GUIDED
	wait4start();

	//create local reference frame 
	initialize_local_frame();

	//request takeoff
	takeoff(2);
	set_destination(0,0,2,0);

	set_destination(25, 0, 2, 0);

	ros::Rate rate(2.0);
	int counter = 0;
	while(ros::ok())
	{
	
		ros::spinOnce();
		rate.sleep();
		
		if (mode_a==0){
			set_destination(25, 0, 2, 0);
			//set_destination(20, 0, mode_h, 0);
			ROS_INFO("Flying to target");			
			if(check_waypoint_reached(.3) == 1)
			{
				ROS_INFO("Target reached");
				land();
				break;
			}		
		}
		else
		{
				ROS_INFO("wait for avoiding");
		}
	}
	
	return 0;
}
