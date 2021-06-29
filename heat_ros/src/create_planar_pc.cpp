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
//    bool 2d = true;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    int n_rows = 100;
    int n_cols = 100;
    int x_dist = 1;
    int y_dist = 1;
    double x_del = x_dist/(double)n_cols;
    double y_del = y_dist/(double)n_rows;
    printf("xdel = %f, ydel = %f", x_del, y_del);
    for(int i=0;i<n_rows;i++){
      for(int j=0;j<n_cols;j++){
//        if (errant_points && ((i*n + j)%7==0)){
//          pcl::PointXYZ pt;
//          pt.x= -1;
//          pt.y = -1;
//          pt.z=0;
//          cloud->push_back(pt);
//        }
//        else{
          pcl::PointXYZ pt;
          pt.x= j*x_del;
          pt.y = i*y_del;
//          if (j%n==0){
            pt.z=0;
//          }
//          else{
//            pt.z=del;
//          }
          cloud->push_back(pt);
//        }
      }
    }


    for (int i=0; i<cloud->size(); i++){
      if (i%n_rows==0){
        printf("\nrow #%d\n", i/n_rows);
      }
      printf("%f, %f\n", cloud->at(i).x, cloud->at(i).y);
    }

    pcl::PLYWriter writer;
    writer.write("/home/cwolfe/planar_cloud_100x100.ply", *cloud);

}
