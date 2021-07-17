#include "ros/ros.h"
#include <cambr_msgs/PlanToolPathsAction.h>
#include <actionlib/client/simple_action_client.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include "pcl_ros/point_cloud.h"


//typedef actionlib::SimpleActionClient<cambr_msgs::PlanToolPathsAction> Client;
static const std::string GENERATE_TOOL_PATHS_ACTION = "generate_heat_tool_paths";
//using GenPathActionClient = actionlib::SimpleActionClient<cambr_msgs::PlanToolPathsAction>;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "heat_surface_plan_client");
  ros::NodeHandle nh;

  std::string filepath = "/home/cwolfe/heat_method_ws/src/Part Meshes/clouds_to_convert/cloud_06-08.ply";
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PLYReader reader;
  reader.read(filepath, *cloud);
  sensor_msgs::PointCloud2 ros_cloud;
  pcl::toROSMsg(*cloud, ros_cloud);
  ROS_INFO_STREAM("cloud size = " << cloud->size());

  actionlib::SimpleActionClient<cambr_msgs::PlanToolPathsAction> client(GENERATE_TOOL_PATHS_ACTION, true);
  ROS_INFO("Client waiting for server");
  client.waitForServer();
  ROS_INFO("Client found server");

  cambr_msgs::PlanToolPathsGoal goal;
//  heat::ProcessConfig process_config = heat::HeatSurfacePlanner::getDefaultConfig();
  cambr_msgs::ToolPathConfig heat_config;
  heat_config.run_density = 0.1;
  heat_config.offset_spacing = 0.2;
  heat_config.height_offset = 0.0;
  heat_config.min_hole_size = 0.2;
  heat_config.min_segment_size = 0.5;

  std::vector<cambr_msgs::ToolPathConfig> configs;
  configs.push_back(heat_config);

  //fill in goal
  goal.cloud = ros_cloud;
  goal.path_configs = configs;

  client.sendGoal(goal);
  client.waitForResult(ros::Duration(5.0));

  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("SUCCESS");


  return 0;
}
