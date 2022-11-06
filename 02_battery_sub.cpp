/**
 *  02 Battery Control
 *  @file 02_battery_sub.cpp
 *  @version 1.0 07.04.2022
 *  @brief Simulation of a drone with battery control
 * 
 *  @author Bernhard Leschinger
 *  Contact: bernhard.leschinger@student.uibk.ac.at
 *
 * 	Drone will be assigned linear waypoints in x direction
 *  and sent to them.
 *  Battery is controlled the whole time by a callback function
 *  subscribed to the ROS battery message
 *  @param
 *  -low battery limit where return to home starts
 *  -very low battery limit, do ann emergency landing
 *  @return -
 *  example: 02_battery_sub 15 5
 *  low battery limit: 15%, return to home
 *  very low battery limit: 5%, do emergency landing
 *  default if no parameters given: 25 20
*/

#include <ros/ros.h>
#include <gnc_functions.hpp>
#include <mavros_msgs/Mavlink.h>
#include <sensor_msgs/BatteryState.h>

// mode_g denotes the flight operations
// 0 - normal flight
// 1 - battery low -> return
// 2 - battery very low -> emergency landing
int mode_g = 0;
// mode for battery control during return flight
int return_g = 0;

float lastpercent=1.00;
// low and very low battery limit
int lbl =0;
int vlbl =0;

//mavros/battery sensor_msgs/BatteryState
// callback function to continuosly parse the battery messages
void battery_cb(const sensor_msgs::BatteryState::ConstPtr& msg)
{	
	//ROS_INFO("%.2f detected %.2f", msg->percentage, msg->percentage*100);
	//ROS_INFO_STREAM(msg->percentage);	
	float batterypercent = msg->percentage;
	// only print battery status if changed to last value
	if (lastpercent!=batterypercent)
	{
		ROS_INFO("battery status: %.2f -> %.2f", batterypercent, batterypercent*100);
		lastpercent=batterypercent;
		// set global mode to "battery low"
		//if (batterypercent <= .25){		
		if (batterypercent*100 <= lbl){
			mode_g = 1;
			ROS_INFO("battery low");
		}
		// set global mode to "battery very low"
		//if (batterypercent <= .20){		
		if (batterypercent*100 <= vlbl){
			mode_g = 2;
			ROS_INFO("battery very low");
		}
	}
}


int main(int argc, char **argv)
{
	if ( (argc!=1) && (argc!=3)) {
		std::cout << "Wrong number of arguments: " << argc << std::endl;
		return 1;
	}

    switch (argc)  
    {
        default:
            printf("Unknown option -%c\n\n", (*argv)[0]);
            break;
        case 1:
            printf("using default parameters\n");
            lbl=25;  // low battery level percentage
            vlbl=20; // very low battery level percentage
            break;
        case 3:
            printf("using given parameters\n");
            //cast strtol to int
            lbl = (int) strtol( argv[1], NULL, 10);
		    vlbl = (int) strtol( argv[2], NULL, 10);
            break;
    }

    printf("Low Battery = %d, Very Low Battery = %d\n",lbl, vlbl);

	//initialize ros
	ros::init(argc, argv, "battery_sub");
	ros::NodeHandle n;
	// subscribe to battery messages, hook callback function
	ros::Subscriber sub = n.subscribe("/mavros/battery", 1, battery_cb);

	//initialize control publisher/subscribers
	init_publisher_subscriber(n);

	// wait for FCU connection
	wait4connect();

	//wait for used to switch to mode GUIDED
	wait4start();

	//create local reference frame 
	initialize_local_frame();

	//request takeoff
	takeoff(5);

	//specify some waypoints, linear on x axis
	std::vector<gnc_api_waypoint> waypointList;
	gnc_api_waypoint nextWayPoint;
	
	for(int i=0; i<5; i++)
	{
		nextWayPoint.x = 100*i;
		nextWayPoint.y = 0;
		nextWayPoint.z = 10;
		nextWayPoint.psi = 0;
		waypointList.push_back(nextWayPoint);	
	}	

	ros::Rate rate(2.0);
	int counter = 0;
	while(ros::ok())
	{
		// normal flight mode
		if (mode_g == 0)
		{
			ros::spinOnce();
			rate.sleep();
			if(check_waypoint_reached(.3) == 1)
			{
				if(counter < waypointList.size())
				{
					ROS_INFO("Waypoint %d", counter);
					set_destination(waypointList[counter].x, waypointList[counter].y, waypointList[counter].z, waypointList[counter].psi);
					counter++;
				}else{
					ROS_INFO("Last Waypoint reached");
					land();
				}
			}
		}

		// battery low flight mode, return to Home Point
		if (mode_g ==1)
		{
			ros::spinOnce();
			rate.sleep();
			// try to return home with low battery
			if (return_g==0)
			{
				return_g=1;
				ROS_INFO("Returning Started");
				set_destination(0, 0, 5, 0);
			}
			// returning started, wait to reach Home Point
			if (return_g==1)
			{
				if(check_waypoint_reached(.3) == 1)
				{
					ROS_INFO("Landing at home ...");
					land();
					break;
				}
			}

		}

		// battery very low flight mode, emergency landing
		if (mode_g ==2)
		{
			ros::spinOnce();
			rate.sleep();
			// immediately landing with very low battery
			ROS_INFO("Emergency Landing ...");
			land();
			break;
		}
	}

	return 0;
}
