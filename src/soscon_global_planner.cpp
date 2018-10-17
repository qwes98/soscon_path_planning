#include <pluginlib/class_list_macros.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Path.h>
#include <soscon_path_planning/soscon_global_planner.h>
#include "coveragepathplanning.hpp"
#include <iostream>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(global_planner::SosconGlobalPlanner, nav_core::BaseGlobalPlanner)

using namespace std;

namespace global_planner
{

SosconGlobalPlanner::SosconGlobalPlanner()
{}


SosconGlobalPlanner::SosconGlobalPlanner(string name, costmap_2d::Costmap2DROS* costmap_ros)
{
	initialize(name, costmap_ros);
}

/*
visualization_msgs::Marker SosconGlobalPlanner::createMarker(const std::string markerName,uint32_t type, geometry_msgs::Pose pose, geometry_msgs::Vector3 scale, std_msgs::ColorRGBA color,  int32_t id, std::string frame_id = std::string("s_map"))
{

      //marker start point
      visualization_msgs::Marker marker;
      marker.header.frame_id = frame_id;
      marker.header.stamp = ros::Time();
      marker.ns = "marker_" + markerName;
      marker.id = id;
      marker.type = type;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = pose.position.x;
      marker.pose.position.y = pose.position.y;
      marker.pose.position.z = pose.position.z;
      marker.pose.orientation.x = pose.orientation.x;
      marker.pose.orientation.y = pose.orientation.y;
      marker.pose.orientation.z = pose.orientation.z;
      marker.pose.orientation.w = pose.orientation.w;
      marker.scale.x = scale.x;
      marker.scale.y = scale.y;
      marker.scale.z = scale.z;
      marker.color.a = color.a;
      marker.color.r = color.r;
      marker.color.g = color.g;
      marker.color.b = color.b;

    return marker;
}
*/


bool SosconGlobalPlanner::getMapFromServer()
{
	ros::Duration(1).sleep();
	map_client_ = nh_.serviceClient<nav_msgs::GetMap>("/static_map");
	nav_msgs::GetMap getMapSrv;
	if (map_client_.call(getMapSrv)) {
		req_msg_.map = getMapSrv.response.map;
		std::cout << "map frame_id: " << req_msg_.map.header.frame_id
				  << std::endl;
	} else {
		ROS_ERROR("Failed to call /static_map service.");
		return false;
	}

	return true;
}

void SosconGlobalPlanner::initializeParam()
{
	// parameters
	req_msg_.erosion_radius = 0.01; // unit: meter
	req_msg_.robot_radius = 0.17; // unit: meter
	req_msg_.occupancy_threshold = 95; // range: 0 ~ 100

	 plan_pub_ = nh_.advertise<nav_msgs::Path>("plan", 1);
}

void SosconGlobalPlanner::prepareStartAndGoal(const geometry_msgs::PoseStamped& start)
{

	//req_msg_.start.header.frame_id = req_msg_.map.header.frame_id;  //  must be the same as map's frame_id

	// start and goal
	req_msg_.start = start;
	req_msg_.start.pose.position.x = 0.75;
	req_msg_.start.pose.position.y = 0.75;

	req_msg_.goal = req_msg_.start;
}

// FIXME: to do refactor
// now not used
/*
void SosconGlobalPlanner::visualization()
{
    ros::Publisher marker_pub =
        n.advertise<visualization_msgs::MarkerArray>("/cleanner_planner", 1);
	ros::Publisher path_pub = 
		n.advertise<nav_msgs::Path>("/path", 1);

    ros::Rate loop_rate(20);

    while (ros::ok())
    {

      int path_size = srv.response.plan.poses.size();

      visualization_msgs::MarkerArray markerArray;

      geometry_msgs::PoseStamped lastPose = srv.response.plan.poses[0];

      //marker start pose
      ////////////////////////////
      geometry_msgs::Pose  plannerStartPose;
      plannerStartPose.position.x = srv.request.start.pose.position.x;
      plannerStartPose.position.y = srv.request.start.pose.position.y;
      plannerStartPose.orientation.w = 1.0;

      geometry_msgs::Vector3 plannerStartScale;
      plannerStartScale.x = 0.2;
      plannerStartScale.y = 0.2;
      plannerStartScale.z = 0.2;

      std_msgs::ColorRGBA plannerStartColor;
      plannerStartColor.a = 1.0;
      plannerStartColor.g = 1.0;

      int32_t plannerStartId = 0;

      visualization_msgs::Marker markerSphereStart = createMarker("PlannerStart",visualization_msgs::Marker::SPHERE,plannerStartPose,plannerStartScale,plannerStartColor,plannerStartId, srv.request.map.header.frame_id);

      markerArray.markers.push_back(markerSphereStart);
      ////////////////////////////


      geometry_msgs::Point lastPoint;
      lastPoint.x = lastPose.pose.position.x;
      lastPoint.y = lastPose.pose.position.y;
      lastPoint.z = 0;

      for (int i = 1; i < path_size; ++i) {

        geometry_msgs::PoseStamped pose = srv.response.plan.poses[i];
        ROS_INFO_STREAM("poses:%s" << pose);

        //marker planner pose
        ////////////////////////////
        geometry_msgs::Pose  markerArrowPose;
        markerArrowPose.position.x = lastPoint.x;
        markerArrowPose.position.y = lastPoint.y;
        markerArrowPose.position.z = 0;
        markerArrowPose.orientation.w = 1.0;

        geometry_msgs::Vector3 markerArrowScale;
        markerArrowScale.x = 0.05;
        markerArrowScale.y = 0.1;
        markerArrowScale.z = 0.1;

        std_msgs::ColorRGBA markerArrowColor;
        markerArrowColor.a = 1.0;
        markerArrowColor.r = 1.0;

        int32_t markerArrowId = i;

        visualization_msgs::Marker markerArrow = createMarker("markerArrow",visualization_msgs::Marker::ARROW,markerArrowPose,markerArrowScale,markerArrowColor,markerArrowId, srv.request.map.header.frame_id); //make markerArrow in map plane

        //arrowHead, arrowEnd
        /////////
        geometry_msgs::Point p;
        p.x = pose.pose.position.x;
        p.y = pose.pose.position.y; p.z = 0; geometry_msgs::Point arrowHeadPoint;
        arrowHeadPoint.x = p.x - lastPoint.x;
        arrowHeadPoint.y = p.y - lastPoint.y;
        arrowHeadPoint.z = 0;


        geometry_msgs::Point arrowEndPoint;
        arrowEndPoint.x = 0;
        arrowEndPoint.y = 0;
        arrowEndPoint.z = 0;

        //markerArrow.points.push_back(arrowHeadPoint);
        //markerArrow.points.push_back(arrowEndPoint);
        /////////
        ////////////////////////////

        markerArrow.points.push_back(arrowEndPoint);
        markerArrow.points.push_back(arrowHeadPoint);

        markerArray.markers.push_back(markerArrow);

        lastPoint = p;

      }

      //marker goal pose
      ////////////////////////////
      geometry_msgs::Pose  plannerGoalPose;
      plannerGoalPose.position.x = srv.request.goal.pose.position.x;
      plannerGoalPose.position.y = srv.request.goal.pose.position.y;
      plannerGoalPose.orientation.w = 1.0;

      geometry_msgs::Vector3 plannerGoalScale;
      plannerGoalScale.x = 0.2;
      plannerGoalScale.y = 0.2;
      plannerGoalScale.z = 0.2;

      std_msgs::ColorRGBA plannerGoalColor;
      plannerGoalColor.a = 1.0;
      plannerGoalColor.b = 1.0;

      int32_t plannerGoalId = path_size + 1;

      visualization_msgs::Marker markerSphereGoal = createMarker("PlannerGoal",visualization_msgs::Marker::SPHERE,plannerGoalPose,plannerGoalScale,plannerGoalColor,plannerGoalId, srv.request.map.header.frame_id);

      markerArray.markers.push_back(markerSphereGoal);
      ////////////////////////////

      marker_pub.publish(markerArray);

      //ROS_INFO_STREAM("markerArrow:" << markerArrow);

	  nav_msgs::Path path = srv.response.plan;
	  path.header.frame_id = srv.request.map.header.frame_id;  //  must be the same as map's frame_id
	  path_pub.publish(path);

      ros::spinOnce();
      loop_rate.sleep();

    }

}
*/


void SosconGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
	if(!initialized_) {
		// initialize other planner parameters
		nh_ = ros::NodeHandle("~/" + name);

		//costmap_ros_ = costmap_ros;
		//costmap_ = costmap_ros_->getCostmap();
		
		getMapFromServer();

		initializeParam();
	
		initialized_ = true;
	} else{
	   ROS_WARN("This planner has already been initialized... doing nothing");
	}
}

void SosconGlobalPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
    if (!initialized_) {
	        ROS_ERROR(
					                "This planner has not been initialized yet, but it is being used, please call initialize() before use");
	        return;
	    }

    //create a message for the plan
	nav_msgs::Path gui_path;
	gui_path.poses.resize(path.size());

	gui_path.header.frame_id = req_msg_.goal.header.frame_id;
	gui_path.header.stamp = ros::Time::now();

	// Extract the plan in world co-ordinates, we assume the path is all in the same frame
	for (unsigned int i = 0; i < path.size(); i++) {
		gui_path.poses[i] = path[i];
	}
	cout << "my path size: " << gui_path.poses.size() << endl;

	plan_pub_.publish(gui_path);
}

void SosconGlobalPlanner::MapToWorld(
    double resolution, double origin_x, double origin_y, unsigned int mx,
    unsigned int my, double *wx, double *wy) {
  *wx = origin_x + (mx + 0.5) * resolution;
  *wy = origin_y + (my + 0.5) * resolution;
}

bool SosconGlobalPlanner::WorldToMap(
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

/*
bool SosconGlobalPlanner::CoveragePlanService(RequestMsg &req, std::vector<geometry_msgs::PoseStamped>& plan) {
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
//   if (0 != (rst = cpp::CoveragePathPlanning(
//                 erosion, start, goal, path,
//                 (req.robot_radius + req.map.info.resolution - 0.01) /
//                     req.map.info.resolution))) {
//
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
*/



bool SosconGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) 
{
if(!initialized_){
ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
return false;
}

	prepareStartAndGoal(goal);

	//plan.clear();
 
	/*
plan.push_back(start);
   for (int i=0; i<20; i++){
        geometry_msgs::PoseStamped new_goal = goal;
        tf::Quaternion goal_quat = tf::createQuaternionFromYaw(1.54);
   
         new_goal.pose.position.x = -2.5+(0.05*i);
         new_goal.pose.position.y = -3.5+(0.05*i);
   
         new_goal.pose.orientation.x = goal_quat.x();
         new_goal.pose.orientation.y = goal_quat.y();
         new_goal.pose.orientation.z = goal_quat.z();
         new_goal.pose.orientation.w = goal_quat.w();
   
      plan.push_back(new_goal);
      }
   plan.push_back(goal);

   return true;
   */
	

	//
  //  build start and goal
  cv::Point start_p, goal_p;
  if (false == WorldToMap(
                   req_msg_.map.info.resolution, req_msg_.map.info.origin.position.x,
                   req_msg_.map.info.origin.position.y, req_msg_.map.info.width,
                   req_msg_.map.info.height, start.pose.position.x,
                   start.pose.position.y, &start_p.x, &start_p.y)) {
    ROS_ERROR("Invalid start. Out of map.");
    return false;
  }
  if (false == WorldToMap(
                   req_msg_.map.info.resolution, req_msg_.map.info.origin.position.x,
                   req_msg_.map.info.origin.position.y, req_msg_.map.info.width,
                   req_msg_.map.info.height, goal.pose.position.x,
                   goal.pose.position.y, &goal_p.x, &goal_p.y)) {
    ROS_ERROR("Invalid goal. Out of map.");
    return false;
  }

  //  build map
  for (int i = 0; i < req_msg_.map.data.size(); ++i) {
    if (-1 == req_msg_.map.data[i]) {  //  replace unknown with 100
      req_msg_.map.data[i] = 100;
    }
  }
  cv::Mat map(
      req_msg_.map.info.height, req_msg_.map.info.width, CV_8UC1, req_msg_.map.data.data());

  //  binarization
  cv::Mat binarization;
  cv::threshold(
      map, binarization, req_msg_.occupancy_threshold, 255, cv::THRESH_BINARY_INV);

  //  erosion
  cv::Mat erosion, element;
  element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::erode(
      binarization, erosion, element, cv::Point(-1, -1),
      (req_msg_.erosion_radius + req_msg_.map.info.resolution - 0.01) /
          req_msg_.map.info.resolution);

  //  coverage path plan
  int rst = -1;
  std::deque<cv::Point> path;
  /*
   if (0 != (rst = cpp::CoveragePathPlanning(
                 erosion, start, goal, path,
                 (req_msg_.robot_radius + req_msg_.map.info.resolution - 0.01) /
                     req_msg_.map.info.resolution))) {
	 */
  if (0 != (rst = cpp::ZigZagPathPlanning(
                erosion, start_p, goal_p, path,
                (req_msg_.robot_radius + req_msg_.map.info.resolution - 0.01) /
                    req_msg_.map.info.resolution))) {
    ROS_ERROR("The planner failed to find a coverage path.");
    ROS_ERROR("Please choose other goal position.");
    return false;
  }

  //  coordinate mapping
  //geometry_msgs::PoseStamped target = start;
  cv::Point cur;

  //  first point
  path.pop_front();
  plan.push_back(start);
  //  not last point
  double target_yaw, next_x, next_y;
  while (path.size() > 1) {
    geometry_msgs::PoseStamped target = goal;
    cur = path.front();
    path.pop_front();

    //  fill position
    MapToWorld(
        req_msg_.map.info.resolution, req_msg_.map.info.origin.position.x,
        req_msg_.map.info.origin.position.y, cur.x, cur.y, &target.pose.position.x,
        &target.pose.position.y);
	//target.pose.position.x = cur.x;
	//target.pose.position.y = cur.y;


    //  fill orientation
    MapToWorld(
        req_msg_.map.info.resolution, req_msg_.map.info.origin.position.x,
        req_msg_.map.info.origin.position.y, path.front().x, path.front().y, &next_x,
        &next_y);
	//next_x = cur.x;
	//next_y = cur.y;


    target_yaw =
        atan2(
            next_y - target.pose.position.y, next_x - target.pose.position.x) *
        180 / CV_PI;
    target.pose.orientation = tf::createQuaternionMsgFromYaw(target_yaw);

	target.header.stamp = ros::Time::now();
	target.header.frame_id = goal.header.frame_id;

    plan.push_back(target);
  }
  //  last point
  path.pop_front();
  plan.push_back(goal);

  cout << "move_base path size: " << plan.size() << endl;


	//CoveragePlanService(req_msg_, plan);
	

	//nav_msgs::Path path;

	/*
	if(planning_server_.CoveragePlanService(req_msg_, plan)) {
		int path_size = path.poses.size();
		for(int i = 0; i < path_size; i++) {
			plan.push_back(path.poses[i]);
		}
		for(int i = 0; i < 10; i++) {
			cout << plan[i] << endl;
		}

		//visualization();
	} else {
		ROS_WARN("The global planner could not get path from planning server");
	}
	*/

	publishPlan(plan);

	/*
	plan.push_back(start);
	   for (int i=0; i<600; i++){
	        geometry_msgs::PoseStamped new_goal = goal;
	        new_goal.pose.orientation = tf::createQuaternionMsgFromYaw(1.54);
	   
	         new_goal.pose.position.x = -2.5+(0.05*i);
	         new_goal.pose.position.y = -3.5+(0.05*i);
	   
	   
	      plan.push_back(new_goal);
	      }
	      plan.push_back(goal);

		  */


	return true;

}

};
