#include <deque>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <soscon_path_planning/request_msg.h>

#ifndef PATH_PLANNER_SERVER_H
#define PATH_PLANNER_SERVER_H

namespace global_planner 
{

class PathPlanningServer
{
public:
	bool CoveragePlanService(RequestMsg &req, std::vector<geometry_msgs::PoseStamped> &plan);

protected:
	void MapToWorld(
    double resolution, double origin_x, double origin_y, unsigned int mx,
    unsigned int my, double *wx, double *wy); 

	bool WorldToMap(
    double resolution, double origin_x, double origin_y, unsigned int size_x,
    unsigned int size_y, double wx, double wy, int *mx, int *my);

};

};

#endif
