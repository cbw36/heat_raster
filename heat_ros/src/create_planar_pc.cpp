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

    bool errant_points = false;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    int n=257;
    double del = 1./n;
    for(int i=0;i<n;i++){
      for(int j=0;j<n;j++){
        if (errant_points && ((i*n + j)%7==0)){
          pcl::PointXYZ pt;
          pt.x= -1;
          pt.y = -1;
          pt.z=0;
          cloud->push_back(pt);
        }
        else{
          pcl::PointXYZ pt;
          pt.x= j*del;
          pt.y = i*del;
          if (j%n==0){
            pt.z=0;
          }
          else{
            pt.z=del;
          }
          cloud->push_back(pt);
        }
      }
    }


    for (int i=0; i<cloud->size(); i++){
      if (i%n==0){
        printf("\nrow #%d\n", i/n);
      }
      printf("%f", cloud->at(i).y);
    }

    pcl::PLYWriter writer;
    writer.write("/home/cwolfe/3d_cloud.ply", *cloud);

}
