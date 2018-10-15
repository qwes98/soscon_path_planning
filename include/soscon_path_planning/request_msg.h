#ifndef REQUEST_MSG_H
#define REQUEST_MSG_H

struct RequestMsg
{
	// parematers
	double erosion_radius;
	double robot_radius;
	char occupancy_threshold;

	// map
	nav_msgs::OccupancyGrid map;

	// start, end point
	geometry_msgs::PoseStamped start, goal;
};

#endif
