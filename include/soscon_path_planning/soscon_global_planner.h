#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <soscon_path_planning/soscon_path_planning_server.h>
#include <soscon_path_planning/request_msg.h>
#include <deque>

using std::string;

#ifndef SOSCON_GLOBAL_PLANNER_H
#define SOSCON_GLOBAL_PLANNER_H

namespace global_planner
{

class PathPlanningServer;

class SosconGlobalPlanner : public nav_core::BaseGlobalPlanner 
{
public:
	SosconGlobalPlanner();
	SosconGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);


	/** overridden classes from interface nav_core::BaseGlobalPlanner **/
	void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
	bool makePlan(const geometry_msgs::PoseStamped& start,
			   const geometry_msgs::PoseStamped& goal,
			   std::vector<geometry_msgs::PoseStamped>& plan
			  );

protected:
	//visualization_msgs::Marker createMarker(const std::string markerName,uint32_t type, geometry_msgs::Pose pose, geometry_msgs::Vector3 scale, std_msgs::ColorRGBA color,  int32_t id, std::string frame_id = std::string("s_map"));
	bool getMapFromServer();
	void initializeParam();
	void prepareStartAndGoal(const geometry_msgs::PoseStamped& start);
	void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);


	//bool CoveragePlanService(RequestMsg &req, std::vector<geometry_msgs::PoseStamped> &plan);

	void MapToWorld(
    double resolution, double origin_x, double origin_y, unsigned int mx,
    unsigned int my, double *wx, double *wy); 

	bool WorldToMap(
    double resolution, double origin_x, double origin_y, unsigned int size_x,
    unsigned int size_y, double wx, double wy, int *mx, int *my);


	//void visualization();

private:
	bool initialized_ = false;
	ros::ServiceClient map_client_;
	ros::NodeHandle nh_;
	ros::Publisher plan_pub_;

	RequestMsg req_msg_;

	//costmap_2d::Costmap2DROS* costmap_ros_;
	//costmap_2d::Costmap2D* costmap_;
	//base_local_planner::WorldModel* world_model_; ///< @brief The world model that the controller will use
};
};
#endif
