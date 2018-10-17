#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>

#ifndef REQUEST_MSG_H
#define REQUEST_MSG_H

struct RequestMsg
{
	// parematers
	double erosion_radius;
	double robot_radius;
	double occupancy_threshold;

	// map
	nav_msgs::OccupancyGrid map;

	// start, end point
	geometry_msgs::PoseStamped start;
	geometry_msgs::PoseStamped goal;
};

#endif
