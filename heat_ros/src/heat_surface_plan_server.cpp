#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <boost/format.hpp>
#include <mutex>
#include <thread>
//#include <heat_msgs/GenerateHeatToolPathsAction.h>
#include <cambr_msgs/PlanToolPathsAction.h>
#include <heat_ros/heat_surface_planner.hpp>
#include <smooth_pose_traj/SmoothPoseTrajectory.h>  // the service to smooth and sample a trajectory

static const std::string GENERATE_TOOL_PATHS_ACTION = "generate_heat_tool_paths";
static const std::string SMOOTH_TOOL_PATHS_SERVICE = "pose_trajectory_smoother";

namespace heat_path_gen
{
using GenPathActionServer = actionlib::SimpleActionServer<cambr_msgs::PlanToolPathsAction>;

class HeatServer
{
public:
  HeatServer(ros::NodeHandle nh, std::string action_name) : nh_(nh), as_(nh, action_name, false)
  {
    path_smooth_client_ = nh_.serviceClient<smooth_pose_traj::SmoothPoseTrajectory>(SMOOTH_TOOL_PATHS_SERVICE);
  }

  ~HeatServer() {}

  void start()
  {
    as_.registerGoalCallback(boost::bind(&HeatServer::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&HeatServer::preemptCB, this));
    as_.start();
  }

protected:
  /** goal has:
   *  sensor_msgs/PointCloud2 cloud   # scan data of blade surface
   *  geometry_msgs/Polygon[] rois    # regions to process
   *  cambr_msgs/ToolPathConfig path_configs # The path configuration parameters for the surface.
   *  result has:
   *  geometry_msgs/PoseArray[] segments  # computed tool path segments
   *  bool tool_path_validities    these indicate which if any of the paths provided are invalid. TODO include these in cambr_msgs?
   **/
  void runHeatPathGeneration(const GenPathActionServer::GoalConstPtr goal)
  {
    ROS_INFO("Starting heat path generation");
    cambr_msgs::PlanToolPathsResult result;
    // result has:
    // bool success                     TODO add to cambr_msgs?
    // geometry_msgs/PoseArray[] paths  # The resulting raster paths
    // bool[] tool_path_validities		  # True when the tool path in 'segments' with the same index is valid.
    std::vector<cambr_msgs::ToolPathConfig> heat_configs; //TODO removed because don't use when create path_gen?
    heat::CloudConfig cloud_config;
    // start lock
    {
      std::lock_guard<std::mutex> lock(goal_process_mutex_);

      // validate data
      if ((goal->cloud.height==0) && (goal->cloud.width==0))
      {
        std::string err_msg = "No cloud was received";
        ROS_ERROR_STREAM(err_msg);
        as_.setAborted(result, err_msg);
        return;
      }

      // verify configs

      // if no rois then plan on whole blade
      if (goal->rois.size() == 0){
        ROS_INFO("No ROIs, plan on whole blade");
        if (goal->path_configs.size() > 1){
          std::string err_msg = "Path configuration array size > 1";
          ROS_ERROR_STREAM(err_msg);
          as_.setAborted(result, err_msg);
        }
        else if (goal->path_configs.size() == 0) {
          ROS_WARN("Path configuration array is empty, using default values");
          heat_configs.resize(1, toCambrMsg(heat::HeatSurfacePlanner::getDefaultConfig()));
        }
        else{
          heat_configs.assign(goal->path_configs.begin(), goal->path_configs.end());
        }
      }

      else{
        ROS_INFO_STREAM(goal->rois.size() << "ROIs, plan on each one and not the whole blade");
        if (goal->path_configs.empty())
        {
          ROS_WARN("Path configuration array is empty, using default values");
          heat_configs.resize(goal->rois.size(), toCambrMsg(heat::HeatSurfacePlanner::getDefaultConfig()));
        }
        else
        {
          heat_configs.assign(goal->path_configs.begin(), goal->path_configs.end());
        }

        // veryfy number of configs & surfaces match
        if (heat_configs.size() != goal->rois.size())
        {
          std::string err_msg = "Number of rois and path configs size have unequal sizes";
          ROS_ERROR_STREAM(err_msg);
          as_.setAborted(result, err_msg);
          return;
        }
      }

      //verify cloud_config params
      if (goal->cloud_config.rows < 1 || goal->cloud_config.cols < 1 ||
          goal->cloud_config.sample_rate < 1 || goal->cloud_config.scaling_factor < 1.0){
        ROS_WARN("Cloud config parameters not provided. Use default");
        cloud_config = heat::HeatSurfacePlanner::getDefaultCloudConfig();
      }
      else{
        cloud_config = toHeatCloudConfig(goal->cloud_config);
      }

    }  // end lock

    // call planner on full surface
    using ToolPath = std::vector<geometry_msgs::PoseArray>;
    int num_toolpaths = std::max((int)goal->rois.size(), 1);
    result.segments.resize(num_toolpaths);
    result.tool_path_validities.resize(num_toolpaths);

    {  // start lock
      std::lock_guard<std::mutex> lock(goal_process_mutex_);
      if (as_.isPreemptRequested())
      {
        ROS_WARN("Canceling Tool Path Generation Request");
//        break;
        return; //TODO used to break here when it was iterating over all ROIs. Should we return instead?
      }
    }  // end lock

    // plan heat paths
    //TODO which config should we use for the full blade? For it to have its own, len(path_config) would have to equal (len(rois)+1)
    heat::ProcessConfig config = toProcessConfig(heat_configs.at(0)); //TODO for now set full surface config to config(0)
    heat::HeatSurfacePlanner path_gen(config);
    std::vector<geometry_msgs::PoseArray> heat_tool_paths;
    path_gen.planPaths(goal->cloud, heat_tool_paths,
                       cloud_config);

    // smooth and resample paths at perscribed point spacing
    smooth_pose_traj::SmoothPoseTrajectory::Request smooth_req;
    smooth_pose_traj::SmoothPoseTrajectory::Response smooth_res;
    std::vector<geometry_msgs::PoseArray> smoothed_tool_paths;
    smooth_req.point_spacing = config.run_density;
    for (int j = 0; j < heat_tool_paths.size(); j++)
    {
      smooth_req.input_poses.poses.assign(heat_tool_paths[j].poses.begin(), heat_tool_paths[j].poses.end());
      if (!path_smooth_client_.call(smooth_req, smooth_res))
      {
        ROS_ERROR("path_smooth_client call failed");
      }
      else
      {
        geometry_msgs::PoseArray PA;
        PA.poses.assign(smooth_res.output_poses.poses.begin(), smooth_res.output_poses.poses.end());
        smoothed_tool_paths.push_back(PA);
      }
    }  // end smooth and re-sampling

    if (smoothed_tool_paths.size() > 0)
    {
      cambr_msgs::HeatToolPath trp;
      for (int j = 0; j < smoothed_tool_paths.size(); j++)  // copy tool paths into the message for return
      {
        geometry_msgs::PoseArray path;
        path.poses.assign(smoothed_tool_paths[j].poses.begin(), smoothed_tool_paths[j].poses.end());
        trp.paths.push_back(path);
      }

      // need to fill in trp.header since its sent to the result, but with what??
      if (goal->rois.size()==0){
        result.segments[0] = move(trp);        
        result.tool_path_validities.at(0) = true;
        ROS_INFO("Full surface processed with %ld paths", result.segments[0].paths.size());
      }
      else{
        base_path_ = move(trp);
      }

    }  // end tool paths size was non-zero for this surface
    else
    {
      ROS_ERROR("Path planning on full surface failed");
      result.success = false;
      return; // TODO should we return here?
    }  // end tool path size was zero for this surface



    //Process each ROI
    for (std::size_t i = 0; i < goal->rois.size(); i++)  // for each roi
    {
      {  // start lock
        std::lock_guard<std::mutex> lock(goal_process_mutex_);
        if (as_.isPreemptRequested())
        {
          ROS_WARN("Canceling Tool Path Generation Request");
          break;
        }
      }  // end lock

      cambr_msgs::HeatToolPath roi_path;
      if (cropPath(goal->cloud, goal->rois[i], roi_path)){
        result.tool_path_validities[i] = true;
        result.segments[i] = move(roi_path);
        ROS_INFO("Surface %ld processed with %ld paths", i, result.segments[i].paths.size());
      }
      else{
        ROS_ERROR("Path planning on surface %ld failed", i);
        if (!goal->proceed_on_failure)
          break;
      }

    } //end for each roi




    // if any paths are invalid report failure in result
    result.success = std::any_of(
        result.tool_path_validities.begin(), result.tool_path_validities.end(), [](const bool& b) { return b; });

    // return result to action
    {  // start lock
      std::lock_guard<std::mutex> lock(goal_process_mutex_);

      if (result.success)
      {
        ROS_INFO("Heat method generated result for %ld meshes", result.segments.size());
        for (size_t jj = 0; jj < result.segments.size(); jj++)
        {
          ROS_INFO("Mesh %ld has %ld paths", jj, result.segments[jj].paths.size());
          for (size_t kk = 0; kk < result.segments[jj].paths.size(); kk++)
          {
            ROS_INFO(
                "Mesh %ld path %ld has %ld waypoints", jj, kk, result.segments[jj].paths[kk].poses.size());
          }
        }

        as_.setSucceeded(result);
      }
      else
      {
        as_.setAborted(result);
      }
    }  // end lock
  }

  void goalCB()
  {
    GenPathActionServer::GoalConstPtr goal;
    {
      std::lock_guard<std::mutex> lock(goal_process_mutex_);

      if (as_.isActive())
      {
        ROS_ERROR("Currently Processing a HEAT request, rejecting");
        return;
      }

      goal = as_.acceptNewGoal();
      if (goal == nullptr)
      {
        std::string err_msg = "Goal ptr received is invalid";
        ROS_ERROR_STREAM(err_msg);
        cambr_msgs::PlanToolPathsResult result;
        as_.setAborted(result, err_msg);
        return;
      }
    }

    runHeatPathGeneration(goal);
  }

  void preemptCB()
  {
    std::lock_guard<std::mutex> lock(goal_process_mutex_);
    as_.setPreempted();
  }


  bool cropPath(const sensor_msgs::PointCloud2& cloud,
                const geometry_msgs::Polygon& roi,
                cambr_msgs::HeatToolPath& paths)
  {
    if (base_path_.paths.size()==0){
      ROS_ERROR("Tried to compute paths on ROIs before computing path on full surface");
      return false;
    }

  }

  cambr_msgs::ToolPathConfig toCambrMsg(const heat::ProcessConfig& tool_config)
  {
    cambr_msgs::ToolPathConfig heat_config_msg ;
    heat_config_msg.run_density = tool_config.run_density;
    heat_config_msg.offset_spacing = tool_config.offset_spacing;
    heat_config_msg.height_offset = tool_config.height_offset;
    heat_config_msg.min_hole_size = tool_config.min_hole_size;
    return std::move(heat_config_msg);
  }

  heat::ProcessConfig toProcessConfig(const cambr_msgs::ToolPathConfig& heat_config_msg)
  {
    heat::ProcessConfig tool_config;
    tool_config.run_density = heat_config_msg.run_density;
    tool_config.offset_spacing = heat_config_msg.offset_spacing;
    tool_config.height_offset = heat_config_msg.height_offset;
    tool_config.min_hole_size = heat_config_msg.min_hole_size;
    return std::move(tool_config);
  }

  heat::CloudConfig toHeatCloudConfig(const cambr_msgs::CloudConfig in_config){
    heat::CloudConfig out_config;
    out_config.rows = in_config.rows;
    out_config.cols = in_config.cols;
    out_config.sample_rate = in_config.sample_rate;
    out_config.scaling_factor = in_config.scaling_factor;
    return out_config;
  }

  ros::NodeHandle nh_;
  GenPathActionServer as_;
  ros::ServiceClient path_smooth_client_;
  std::mutex goal_process_mutex_;
  cambr_msgs::HeatToolPath base_path_; // TOOD use this or geometry_msgs::PoseArray

};

}  // namespace heat_path_gen

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_generator");
  ros::AsyncSpinner spinner(4);
  ros::NodeHandle nh;
  spinner.start();
  heat_path_gen::HeatServer heat_server(nh, GENERATE_TOOL_PATHS_ACTION);
  heat_server.start();
  ros::waitForShutdown();
  return 0;
}
