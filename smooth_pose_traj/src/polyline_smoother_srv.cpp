#include <ros/ros.h>
#include <boost/format.hpp>
#include <mutex>
#include <thread>
#include <heat_msgs/PolylineSmoother.h> // the service
#include <heat_msgs/SmoothPoseTrajectory.h>
#include <smooth_pose_traj/polyline_smoother.hpp> // the .hpp for this cpp file
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

static const std::string POLYLINE_SMOOTHER = "polyline_smoother";
static const std::string SMOOTH_TOOL_PATHS_SERVICE = "pose_trajectory_smoother";


/*
bool add(beginner_tutorials::AddTwoInts::Request  &req,
         beginner_tutorials::AddTwoInts::Response &res)
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}
*/
namespace PolylineSmoother
{
class PolylineSmootherSrv
{
public:
  PolylineSmootherSrv(ros::NodeHandle nh, std::string server_name):
    nh_(nh)
  {
    polyline_smoother_srv_ = nh_.advertiseService(POLYLINE_SMOOTHER, &PolylineSmootherSrv::polylineSmootherCB, this);
    path_smooth_client_ = nh_.serviceClient<heat_msgs::SmoothPoseTrajectory>(SMOOTH_TOOL_PATHS_SERVICE);

  }

  ~PolylineSmootherSrv()
  {

  }

  bool polylineSmootherCB(heat_msgs::PolylineSmoother::Request& req, heat_msgs::PolylineSmoother::Response& res)
  {
    // Step 1: Convert vertices to quaternion
    // Step 2: Smooth it using the same method as PoseTrajSmoother
    PolylineSmoother PS(req.pt_spacing, req.surface_mesh, req.source_indices);
    PS.compute_pose_arrays();

    // smooth and resample paths at perscribed point spacing
    heat_msgs::SmoothPoseTrajectory::Request smooth_req;
    heat_msgs::SmoothPoseTrajectory::Response smooth_res;
    smooth_req.pt_spacing = req.pt_spacing;
    geometry_msgs::PoseArray pa = PS.pose_arrays_;
    smooth_req.input_poses = PS.pose_arrays_;
    bool result = path_smooth_client_.call(smooth_req, smooth_res);
    res.output_poses = smooth_res.output_poses;
    return result;
  }

  ros::ServiceServer polyline_smoother_srv_;
  ros::ServiceClient path_smooth_client_;
  ros::NodeHandle nh_;
};

} //end PolylineSmoother Namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "polyline_smoother_server");
  ros::AsyncSpinner spinner(4);
  ros::NodeHandle nh;
  spinner.start();
  PolylineSmoother::PolylineSmootherSrv PSS(nh, POLYLINE_SMOOTHER);
  ros::waitForShutdown();
  return 0;
}
