#include <soscon_path_planning/soscon_path_planning_server.h>
#include "coveragepathplanning.hpp"
#include <iostream>
#include <tf/tf.h>
using namespace std;

namespace global_planner
{

void PathPlanningServer::MapToWorld(
    double resolution, double origin_x, double origin_y, unsigned int mx,
    unsigned int my, double *wx, double *wy) {
  *wx = origin_x + (mx + 0.5) * resolution;
  *wy = origin_y + (my + 0.5) * resolution;
}

bool PathPlanningServer::WorldToMap(
    double resolution, double origin_x, double origin_y, unsigned int size_x,
    unsigned int size_y, double wx, double wy, int *mx, int *my) {
  if (wx < origin_x || wy < origin_y)
    return false;

  *mx = static_cast<int>((wx - origin_x) / resolution);
  *my = static_cast<int>((wy - origin_y) / resolution);

  if (*mx < size_x && *my < size_y)
    return true;

  return false;
}

bool PathPlanningServer::CoveragePlanService(RequestMsg &req, std::vector<geometry_msgs::PoseStamped>& plan) {

	// 나의 plan_pub은 publish했는데 move base는 안함
  plan.push_back(req.start);
     for (int i=0; i<20; i++){
	      geometry_msgs::PoseStamped new_goal = req.goal;
	      tf::Quaternion goal_quat = tf::createQuaternionFromYaw(1.54);
	 
	       new_goal.pose.position.x = -2.5+(0.05*i);
	       new_goal.pose.position.y = -3.5+(0.05*i);
	 
	       new_goal.pose.orientation.x = goal_quat.x();
	       new_goal.pose.orientation.y = goal_quat.y();
	       new_goal.pose.orientation.z = goal_quat.z();
	       new_goal.pose.orientation.w = goal_quat.w();
	 
	    plan.push_back(new_goal);
	    }
	    plan.push_back(req.goal);

		return true;

  if (req.erosion_radius < 0) {
    ROS_ERROR("erosion_radius < 0");
    return false;
  }
  if (req.robot_radius < 0) {
    ROS_ERROR("robot_radius < 0");
    return false;
  }
  if (req.occupancy_threshold < 0 || req.occupancy_threshold > 100) {
    ROS_ERROR("occupancy_threshold out of range 0~100");
    return false;
  }
  if (req.start.header.frame_id != req.map.header.frame_id) {
    ROS_ERROR("start's frame_id != map's frame_id");
    return false;
  }
  if (req.goal.header.frame_id != req.map.header.frame_id) {
    ROS_ERROR("goal's frame_id != map's frame_id");
    return false;
  }
  if (req.map.info.resolution < 0) {
    ROS_ERROR("invalid map: resolution < 0");
    return false;
  }
  if (req.map.info.width * req.map.info.height < 1) {
    ROS_ERROR("invalid map: width * height < 1");
    return false;
  }
  if (req.map.info.width * req.map.info.height != req.map.data.size()) {
    ROS_ERROR("invalid map: width * height != data size");
    return false;
  } 
  //  build start and goal
  cv::Point start, goal;
  if (false == WorldToMap(
                   req.map.info.resolution, req.map.info.origin.position.x,
                   req.map.info.origin.position.y, req.map.info.width,
                   req.map.info.height, req.start.pose.position.x,
                   req.start.pose.position.y, &start.x, &start.y)) {
    ROS_ERROR("Invalid start. Out of map.");
    return false;
  }
  if (false == WorldToMap(
                   req.map.info.resolution, req.map.info.origin.position.x,
                   req.map.info.origin.position.y, req.map.info.width,
                   req.map.info.height, req.goal.pose.position.x,
                   req.goal.pose.position.y, &goal.x, &goal.y)) {
    ROS_ERROR("Invalid goal. Out of map.");
    return false;
  }


  //  build map
  for (int i = 0; i < req.map.data.size(); ++i) {
    if (-1 == req.map.data[i]) {  //  replace unknown with 100
      req.map.data[i] = 100;
    }
  }
  cv::Mat map(
      req.map.info.height, req.map.info.width, CV_8UC1, req.map.data.data());

  //  binarization
  cv::Mat binarization;
  cv::threshold(
      map, binarization, req.occupancy_threshold, 255, cv::THRESH_BINARY_INV);

  //  erosion
  cv::Mat erosion, element;
  element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::erode(
      binarization, erosion, element, cv::Point(-1, -1),
      (req.erosion_radius + req.map.info.resolution - 0.01) /
          req.map.info.resolution);

  //  coverage path plan
  int rst = -1;
  std::deque<cv::Point> path;
  /*
   if (0 != (rst = cpp::CoveragePathPlanning(
                 erosion, start, goal, path,
                 (req.robot_radius + req.map.info.resolution - 0.01) /
                     req.map.info.resolution))) {
	 */
  if (0 != (rst = cpp::ZigZagPathPlanning(
                erosion, start, goal, path,
                (req.robot_radius + req.map.info.resolution - 0.01) /
                    req.map.info.resolution))) {
    ROS_ERROR("The planner failed to find a coverage path.");
    ROS_ERROR("Please choose other goal position.");
    return false;
  }

  //  coordinate mapping
  //geometry_msgs::PoseStamped target = req.start;
  cv::Point cur;

  //  first point
  path.pop_front();
  plan.push_back(req.start);
  //  not last point
  double target_yaw, next_x, next_y;
  while (path.size() > 1) {
    geometry_msgs::PoseStamped target = req.goal;
    cur = path.front();
    path.pop_front();

    //  fill position
    MapToWorld(
        req.map.info.resolution, req.map.info.origin.position.x,
        req.map.info.origin.position.y, cur.x, cur.y, &target.pose.position.x,
        &target.pose.position.y);
	//target.pose.position.x = cur.x;
	//target.pose.position.y = cur.y;


    //  fill orientation
    MapToWorld(
        req.map.info.resolution, req.map.info.origin.position.x,
        req.map.info.origin.position.y, path.front().x, path.front().y, &next_x,
        &next_y);
	//next_x = cur.x;
	//next_y = cur.y;


    target_yaw =
        atan2(
            next_y - target.pose.position.y, next_x - target.pose.position.x) *
        180 / CV_PI;
    target.pose.orientation = tf::createQuaternionMsgFromYaw(target_yaw);

	//target.header.stamp = ros::Time::now();

    plan.push_back(target);
  }
  //  last point
  path.pop_front();
  plan.push_back(req.goal);

  return true;
}

};
