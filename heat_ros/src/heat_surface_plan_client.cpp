#include "ros/ros.h"
#include <heat_msgs/GenerateHeatToolPathsAction.h>
#include <actionlib/client/simple_action_client.h>
//#include <heat_ros/heat_surface_planner.hpp>
#include <opp_gui/utils.h>

//typedef actionlib::SimpleActionClient<cambr_msgs::PlanToolPathsAction> Client;
static const std::string GENERATE_TOOL_PATHS_ACTION = "generate_heat_tool_paths";
//using GenPathActionClient = actionlib::SimpleActionClient<cambr_msgs::PlanToolPathsAction>;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "heat_surface_plan_client");
  ros::NodeHandle nh;

  actionlib::SimpleActionClient<heat_msgs::GenerateHeatToolPathsAction> client(GENERATE_TOOL_PATHS_ACTION, true);
  ROS_INFO("Client waiting for server");
  client.waitForServer();
  ROS_INFO("Client found server");

  heat_msgs::GenerateHeatToolPathsGoal goal;
//  heat::ProcessConfig process_config = heat::HeatSurfacePlanner::getDefaultConfig();
  heat_msgs::HeatRasterGeneratorConfig heat_config;
  heat_config.point_spacing = 0.1;
  heat_config.raster_spacing = 0.2;
  heat_config.tool_offset = 0.0;
  heat_config.min_hole_size = 0.2;
  heat_config.min_segment_size = 0.5;
  heat_config.raster_rot_offset = 0.0;

  std::vector<heat_msgs::HeatRasterGeneratorConfig> configs;
  configs.push_back(heat_config);

  //get mesh from file
  std::string resource = "file:///home/cwolfe/heat_method_ws/src/Part Meshes/meshes_from_clouds/subsampled_mesh_06-08.ply";
  shape_msgs::Mesh mesh_msg;
  opp_gui::utils::getMeshMsgFromResource(resource, mesh_msg);

  //fill in goal
  std::vector<shape_msgs::Mesh> meshes;
  meshes.push_back(mesh_msg);
  goal.surface_meshes = meshes;
  goal.path_configs = configs;

  client.sendGoal(goal);
  client.waitForResult(ros::Duration(5.0));

  if (client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("SUCCESS");


  return 0;
}
