#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/meshio.h"
#include "geometrycentral/surface/surface_mesh.h"
#include "geometrycentral/utilities/vector3.h"

#include "geometrycentral/surface/heat_method_distance.h"
#include <heat_ros/heat_surface_planner.hpp>
#include <geometrycentral/surface/simple_polygon_mesh.h>
#include "geometrycentral/surface/surface_mesh_factories.h"

#include "geometrycentral/pointcloud/point_cloud.h"
#include "geometrycentral/pointcloud/point_position_geometry.h"
#include "geometrycentral/pointcloud/point_cloud_heat_solver.h"
#include "geometrycentral/pointcloud/point_cloud_io.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include "pcl_ros/point_cloud.h"

#include "ros/ros.h"

using namespace geometrycentral;
using namespace geometrycentral::surface;
using namespace geometrycentral::pointcloud;


int main(int argc, char** argv) {

    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

//    // Load a general surface mesh from file
//    std::unique_ptr<SurfaceMesh> mesh;
//    std::unique_ptr<VertexPositionGeometry> geometry;
//    int n = 100;

//    Eigen::MatrixXd vMat(10000,3);
//    Eigen::MatrixXi fMat(20000, 3);
//    for (int i=0; i<n; i++){
//      for (int j=0; j<n; j++){
//        vMat(i*n + j, 0) = i*0.001;
//        vMat(i*n + j, 1) = j*0.001;
//        vMat(i*n + j, 2) = 0.0;
//      }
//   }

//    for (int i=0; i<n-1; i++){
//      for (int j=0; j<n-1; j++){
//        fMat(i*n + j, 0) = i*n + j;
//        fMat(i*n + j, 1) = i*n + j+1;
//        fMat(i*n + j, 2) = (i+1)*n + j;

//        fMat(i*n + j + 1, 0) = i*n + j+1;
//        fMat(i*n + j + 1, 1) = (i+1)*n + j;
//        fMat(i*n + j + 1, 2) = (i+1)*n + j+1;
//      }
//   }
//    std::tie(mesh, geometry) = makeSurfaceMeshAndGeometry(vMat, fMat);

    std::string filepath;
    nh.getParam("/pc_to_mesh/filepath", filepath);
    ROS_INFO_STREAM(filepath);

    std::unique_ptr<SurfaceMesh> mesh;
    std::unique_ptr<VertexPositionGeometry> mesh_geom;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PLYReader reader;
    reader.read(filepath, *cloud);

    ROS_INFO_STREAM("height = " << cloud->height << ", width = " << cloud->width << ", points = " << cloud->size());
    int num_valid = 0;
    double min_y = 0.0;
    for (int i=0; i<cloud->size(); i++){
      pcl::PointXYZ pt = cloud->at(i);
      if (pt.x!=0 && pt.y!=0)
        num_valid++;
      ROS_INFO_STREAM("x = " << pt.x << ", y = " << pt.y);
    }
    ROS_ERROR_STREAM("num valid pts = " << num_valid);

//    sensor_msgs::PointCloud2 cloud_;
    ros::Publisher cloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("cloud", 1);;
//    pcl::toROSMsg(*cloud, cloud_);
//    cloud->header.stamp = ros::Time::now();
    cloud->header.frame_id = "map";

    cloud_pub.publish(*cloud);

    ros::Rate rate(0.2);
    while (ros::ok()){
//      cloud.header.stamp = ros::Time::now();
      cloud_pub.publish(*cloud);
      rate.sleep();
      ROS_INFO("publish");
    }


//    sensor_msgs::PointCloud2 cloud_ros;
//    pcl::toROSMsg(cloud_ros, cloud);
}
