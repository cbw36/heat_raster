#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include "pcl_ros/point_cloud.h"

#include <visualization_msgs/Marker.h>
#include <geometric_shapes/shape_to_marker.h>

#include "ros/ros.h"

using namespace geometrycentral;
using namespace geometrycentral::surface;
using namespace geometrycentral::pointcloud;


int main(int argc, char** argv) {


  //NOT WORKING!!!! IGNORE
  ros::init(argc, argv, "main");
  ros::NodeHandle nh;
  shape_msgs::Mesh mesh_;
  geometry_msgs::Point point;
  geometry_msgs::Point point2;
  geometry_msgs::Point point3;
  point.x=0; point.y=0; point.z=0;
  point2.x=1; point2.y=1; point2.z=1;
  point3.x=2; point3.y=2; point3.z=2;

  mesh_.vertices.push_back(point); mesh_.vertices.push_back(point2); mesh_.vertices.push_back(point3);
  shape_msgs::MeshTriangle mesh_triangle;
  mesh_triangle.vertex_indices = {0, 1, 2};
  mesh_.triangles.push_back(mesh_triangle);

  visualization_msgs::Marker marker;
  geometric_shapes::constructMarkerFromShape(mesh_, marker, true);

  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("marker", 1);
  ros::Rate rate(0.2);
  while (ros::ok()){
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "map";
    marker_pub.publish(marker);
    rate.sleep();
    ROS_INFO("publish");
  }
}
