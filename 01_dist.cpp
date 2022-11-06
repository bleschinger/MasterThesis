/**
 *  01 Distance Control
 *  @file 01_dist.cpp
 *  @version 1.0 30.03.2022
 *  @brief Simulation of a drone with distance control
 * 
 *  @author Bernhard Leschinger
 *  Contact: bernhard.leschinger@student.uibk.ac.at
 *
 * 	Drone will be assigned random waypoints and sent to them
 *  If waypoint would be further away than the safety distance
 *  the drone will be sent back to the homepoint
 *  @param
 *  -number of random waypoints
 *  -limit for random distance to generate waypoints
 *  -safety distance
 *  @return -
 *  example: 01_dist 10 20 12
 *  do 10 runs
 *  maximum waypoint distance 20 is +10/-10 for x and y
 *  z will be from 1 to 11 (distance/2 +1)
 *  distance limit is 12.
 *  default if no parameters given: 5 10 4
*/

#include <ros/ros.h>
#include <gnc_functions.hpp>

int main(int argc, char** argv)
{
	if ( (argc!=1) && (argc!=4)) {
		std::cout << "Wrong number of arguments: " << argc << std::endl;
		return 1;
	}

    int N = 0; // number of runs
    int D = 0; // max random distance x,y: 1/2 for +direction and 1/2 for -direction
    int S = 0; // safety distance

	int counter = 1;
	int dist_x = 0;
	int dist_y = 0;
	int dist_z = 0;
	float dist_c = 0;
	/* initialize random seed: */
	srand ( time(NULL) );

    switch (argc)  
    {
        default:
            printf("Unknown option -%c\n\n", (*argv)[0]);
            break;
        case 1:
            printf("using default parameters\n");
            N=5;  // number of runs
            D=10; // max random distance x,y: 1/2 for +direction and 1/2 for -direction
            S=4;  // safety distance
            break;
        case 4:
            printf("using given parameters\n");
            //cast strtol to int
            N = (int) strtol( argv[1], NULL, 10);
		    D = (int) strtol( argv[2], NULL, 10);
		    S = (int) strtol( argv[3], NULL, 10);
            break;
    }

    printf("N = %d, D = %d, S = %d\n",N,D,S);

	//initialize ros 
	ros::init(argc, argv, "gnc_node");
	ros::NodeHandle gnc_node("~");
	
	//initialize control publisher/subscribers
	init_publisher_subscriber(gnc_node);

  	// wait for FCU connection
	wait4connect();

	//wait for used to switch to mode GUIDED
	wait4start();

	//create local reference frame 
	initialize_local_frame();

	//request takeoff
	takeoff(3);

	//specify control loop rate.
	ros::Rate rate(2.0);

	//set_speed(150.00);
	
	while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
		
		// check if waypoint is already reached, with a tolerance
		if(check_waypoint_reached(.3) == 1)
		{
			if (counter < N+1)
			{
				printf("counter: %d\n", counter);
				// x and y = +/- distance/2: 10-> -5 to +5
				dist_x=rand()%D-(int)(D/2);
				dist_y=rand()%D-(int)(D/2);
				// z is distance/2 starting at 1: 10-> 1 to 5
				dist_z=(int)(rand()%D)/2+1;
				// calculate 3D distance of new waypoint to Home Point (0,0,0)
				dist_c = sqrt(dist_x*dist_x + dist_y*dist_y + dist_z*dist_z);

				// new waypoint would be too far away, more than safety distance S
				if (dist_c > S) {
					// Safety measure - waypoint out of range
				 	printf("fly to: x %d, y %d, z %d, distance: %2.4f\n", dist_x, dist_y, dist_z, dist_c);
					printf("SAFETY MEASURE - distance warning! Waypoint out of range!\n");
					printf("fly to: x %d, y %d, z %d, distance: %2.4f\n", 0, 0, 6, 0.0);
					set_destination(0, 0, 6, 0);
				 } else {
				 	// distance OK, fly to waypoint in range
				 	printf("fly to: x %d, y %d, z %d, distance: %2.4f\n", dist_x, dist_y, dist_z, dist_c);
				 	set_destination(dist_x, dist_y, dist_z, -1);
				}

				counter++;
				if (counter>=N+1) {
								printf("counter END, flying home.\n");
								set_destination(0, 0, 2, 0); }
			}else{
				//land after N waypoints generated, land at Home Point
				printf("landing mode\n");
				land();
				break;
			}	
		}	
		
	}
	return 0;
}
