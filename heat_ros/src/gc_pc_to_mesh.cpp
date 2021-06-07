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

#include <visualization_msgs/Marker.h>
#include <geometric_shapes/shape_to_marker.h>

#include "ros/ros.h"

using namespace geometrycentral;
using namespace geometrycentral::surface;
using namespace geometrycentral::pointcloud;


int main(int argc, char** argv) {

    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

/*
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
*/

    std::string filepath;
    int rows, cols;
    nh.getParam("/gc_pc_to_mesh/filepath", filepath);
    nh.getParam("/gc_pc_to_mesh/rows", rows);
    nh.getParam("/gc_pc_to_mesh/cols", cols);
    ROS_INFO_STREAM(filepath);

    std::unique_ptr<SurfaceMesh> mesh;
    std::unique_ptr<VertexPositionGeometry> geometry;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PLYReader reader;
    reader.read(filepath, *cloud);

    Eigen::MatrixXd vMat(cloud->size(), 3);
    Eigen::MatrixXi fMat(cloud->size()*2, 3);
    std::vector<int> ind_map;

    ROS_INFO_STREAM("number of points = " << cloud->size());

    int num_valid_pts = 0;
    for (int i=0; i<cloud->size(); i++){
      pcl::PointXYZ pt = cloud->at(i);
//      ROS_INFO_STREAM("x=" << pt.x <<", y="<<pt.y<<", z="<<pt.z);
      if (pt.x>=0 && pt.y>=0 && pt.z>=0){
        vMat(num_valid_pts, 0) = pt.x;
        vMat(num_valid_pts, 1) = pt.y;
        vMat(num_valid_pts, 2) = pt.z;
        ind_map.push_back(num_valid_pts);
        num_valid_pts ++;
      }
      else{
        ind_map.push_back(-1);
      }
    }

    ROS_INFO_STREAM("FINISH VERTICES. Total num = " << num_valid_pts);
    int num_triangles = 0;
    int num_discarded_triangles = 0;
    for (int i=0; i<rows-1; i++){
      for (int j=0; j<cols-1; j++){
        int ind_1 = i*cols + j;
        int ind_2 = (i+1)*cols + j;
        int ind_3 = i*cols + j+1;
        if ((ind_map.at(ind_1) != -1) && (ind_map.at(ind_2) != -1) && (ind_map.at(ind_3) != -1)){
          fMat(num_triangles, 0) = ind_map.at(ind_1);
          fMat(num_triangles, 1) = ind_map.at(ind_2);
          fMat(num_triangles, 2) = ind_map.at(ind_3);
          num_triangles++;
        }
        else {
          num_discarded_triangles++;
        }

        int ind_4 = i*cols + j+1;
        int ind_5 = (i+1)*cols + j;
        int ind_6 = (i+1)*cols + j+1;
        if ((ind_map.at(ind_4) != -1) && (ind_map.at(ind_5) != -1) && (ind_map.at(ind_6) != -1)){
          fMat(num_triangles, 0) = ind_map.at(ind_4);
          fMat(num_triangles, 1) = ind_map.at(ind_5);
          fMat(num_triangles, 2) = ind_map.at(ind_6);
          num_triangles++;
        }
        else {
          num_discarded_triangles++;
        }
      }
    }

    ROS_INFO_STREAM("finish TRIANGLES. total num = " << num_triangles << ", num_discarded = " << num_discarded_triangles);

    Eigen::MatrixXd reduced_vMat(num_valid_pts, 3);
    reduced_vMat = vMat.block(0, 0, num_valid_pts, 3);

    Eigen::MatrixXi reduced_fMat(num_triangles, 3);
    reduced_fMat = fMat.block(0, 0, num_triangles, 3);

    for (int i=0; i<num_valid_pts; i++){
//      ROS_INFO_STREAM("VMAT x = " << reduced_vMat(i,0) <<
//                      ", y = " << reduced_vMat(i,1) <<
//                      ", z = " << reduced_vMat(i,2));
      if (reduced_vMat(i,0) != vMat(i,0) ||
          reduced_vMat(i,1) != vMat(i,1) ||
          reduced_vMat(i,2) != vMat(i,2) ){
        ROS_INFO("reduced vmat != vmat");
      }
    }

    for (int i=0; i<num_triangles; i++){
//      ROS_INFO_STREAM("FMAT pt 1 = " << reduced_fMat(i,0) <<
//                      ", pt 2 = " << reduced_fMat(i,1) <<
//                      ", pt 3 = " << reduced_fMat(i,2));
      if (reduced_fMat(i,0) != fMat(i,0) ||
          reduced_fMat(i,1) != fMat(i,1) ||
          reduced_fMat(i,2) != fMat(i,2) ){
        ROS_INFO("reduced fmat != fmat");
      }
    }

    ROS_INFO("CREATE MESH AND GEOMETRY");
    ROS_ERROR("BLADE FAILS BECAUSE THERE ARE VERTICES NOT CONNECTED TO ANY TRIANGLES. WOULD HAVE TO REMOVE THESE VERTICES BEFORE MAKING TRIANGLES");
    std::tie(mesh, geometry) = makeSurfaceMeshAndGeometry(reduced_vMat, reduced_fMat);
    writeSurfaceMesh(*mesh, *geometry, "/home/cwolfe/gc_shiny_side_bottom_mesh.obj");
}
