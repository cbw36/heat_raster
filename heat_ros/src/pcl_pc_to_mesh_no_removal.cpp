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

int main(int argc, char** argv) {

    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    std::vector<pcl::Vertices> polys;

    pcl::PolygonMesh mesh;

    std::string filepath;
    int rows, cols;
    nh.getParam("/pcl_pc_to_mesh_no_removal/filepath", filepath);
    nh.getParam("/pcl_pc_to_mesh_no_removal/rows", rows);
    nh.getParam("/pcl_pc_to_mesh_no_removal/cols", cols);
    ROS_INFO_STREAM(filepath);


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PLYReader reader;
    reader.read(filepath, *cloud);

    std::vector<int> ind_map;

    ROS_INFO_STREAM("number of points = " << cloud->size());

    for (int i=0; i<rows-1; i++){
      for (int j=0; j<cols-1; j++){
        int ind_1 = i*cols + j;
        int ind_2 = (i+1)*cols + j;
        int ind_3 = i*cols + j+1;
        pcl::Vertices vertices;
        vertices.vertices.push_back(ind_1);
        vertices.vertices.push_back(ind_2);
        vertices.vertices.push_back(ind_3);
        polys.push_back(vertices);

        int ind_4 = i*cols + j+1;
        int ind_5 = (i+1)*cols + j;
        int ind_6 = (i+1)*cols + j+1;
        pcl::Vertices vertices2;
        vertices2.vertices.push_back(ind_4);
        vertices2.vertices.push_back(ind_5);
        vertices2.vertices.push_back(ind_6);
        polys.push_back(vertices2);
      }
    }

    pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2);
    pcl::toPCLPointCloud2(*cloud, *cloud_blob);

    mesh.polygons = polys;
    mesh.cloud = *cloud_blob;

    std::string ply_filename("/home/cwolfe/heat_method_ws/src/Part Meshes/meshes_from_clouds/planar_cloud_69x188.ply");
    pcl::io::savePLYFile(ply_filename, mesh);

}
