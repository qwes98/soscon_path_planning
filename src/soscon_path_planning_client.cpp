#include <nav_msgs/GetMap.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "coverage_path_planning/GetCoveragePath.h"

// Example:
//////////////////
// geometry_msgs::Pose  plannerGoal;
// plannerGoal.position.x = srv.request.goal.pose.position.x;
// plannerGoal.position.y = srv.request.goal.pose.position.y;
// plannerGoal.orientation.w = 1.0;
//
// geometry_msgs::Vector3 plannerGoalScale;
// plannerGoalScale.x = 0.2;
// plannerGoalScale.y = 0.2;
// plannerGoalScale.z = 0.2;
//
// std_msgs::ColorRGBA plannerGoalColor;
// plannerGoalColor.a = 1.0;
// plannerGoalColor.b = 1.0;
//
// int32_t plannerGoalId = path_size + 1;
//
// visualization_msgs::Marker markerSphereGoal = createMarker("PlannerGoal",visualization_msgs::Marker::SPHERE,plannerGoal,plannerGoalScale,plannerGoalColor,plannerGoalId, srv.request.map.header.frame_id);
//////////////////
visualization_msgs::Marker createMarker(const std::string markerName,uint32_t type, geometry_msgs::Pose pose, geometry_msgs::Vector3 scale, std_msgs::ColorRGBA color,  int32_t id, std::string frame_id = std::string("s_map"))
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


int main(int argc, char **argv) {
  ros::init(argc, argv, "coverage_path_planning_client");

  ros::NodeHandle n;
  ros::ServiceClient client =
      n.serviceClient<coverage_path_planning::GetCoveragePath>(
          "/sweeper/make_coverage_plan");

  coverage_path_planning::GetCoveragePath srv;

  srv.request.erosion_radius = 0.01;     //  unit: meter
  srv.request.robot_radius = 0.17;        //  unit: meter
  srv.request.occupancy_threshold = 95;  //  range:0~100

  //  get map from mapserver
  ros::Duration(1).sleep();
  ros::ServiceClient mapClient =
      n.serviceClient<nav_msgs::GetMap>("/static_map");
  nav_msgs::GetMap getMapSrv;
  if (mapClient.call(getMapSrv)) {
    srv.request.map = getMapSrv.response.map;
    std::cout << "map frame_id: " << srv.request.map.header.frame_id
              << std::endl;
  } else {
    ROS_ERROR("Failed to call /static_map service.");
    return 1;
  }

  //  prepare start and goal
  geometry_msgs::PoseStamped start;
  start.header.frame_id =
      srv.request.map.header.frame_id;  //  must be the same as map's frame_id
  start.pose.position.x = 0.75;
  //start.pose.position.y = -0.10;
  start.pose.position.y = 0.75;
  srv.request.start = start;
  std::cout << "start frame_id: " << srv.request.start.header.frame_id
            << std::endl;
  srv.request.goal = start;
  std::cout << "goal frame_id: " << srv.request.goal.header.frame_id
            << std::endl;

  if (client.call(srv)) {  //  draw the path on rviz
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
        p.y = pose.pose.position.y;
        p.z = 0;

        geometry_msgs::Point arrowHeadPoint;
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
  } else {
    ROS_ERROR("Failed to call service /sweeper/make_coverage_plan");
    return 1;
  }

  //    ros::spin();

  return 0;
}
