#include <pluginlib/class_list_macros.h>
#include <nav_msgs/GetMap.h>
#include <soscon_path_planning/soscon_global_planner.h>

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


bool SosconGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) 
{
	if(!initialized_){
		ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
	    return false;
	}

	prepareStartAndGoal(goal);

	//nav_msgs::Path path;

	if(planning_server_.CoveragePlanService(req_msg_, plan)) {
		/*
		int path_size = path.poses.size();
		for(int i = 0; i < path_size; i++) {
			plan.push_back(path.poses[i]);
		}
*/

		//visualization();
	} else {
		ROS_WARN("The global planner could not get path from planning server");
	}

	return true;

}

};
