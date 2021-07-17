/**
 * @file heat_surface_planner.h
 * @copyright Copyright (c) 2020, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef INCLUDE_HEAT_SURFACE_PLANNER_H
#define INCLUDE_HEAT_SURFACE_PLANNER_H

#include <shape_msgs/Mesh.h>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <eigen3/Eigen/Eigen>

//extern "C" {
//#include "libgeodesic/hmTriDistance.h"
//#include "libgeodesic/hmContext.h"
//#include "libgeodesic/hmUtility.h"
//#include "libgeodesic/hmVectorSizeT.h"
//}
//#include "libgeodesic/hmHeatPath.hpp"
#include "geometrycentral/surface/heat_method_distance.h"
#include "geometrycentral/surface/meshio.h"
#include "geometrycentral/surface/surface_mesh.h"

#include "heat_ros/hm_heat_path.hpp"

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Polygon.h>

#include <sensor_msgs/PointCloud2.h>

namespace heat
{

struct ProcessPath
{
  std::vector<geometry_msgs::Pose> poses;
};

struct ProcessConfig
{
  float run_density; //spacing between points on a single segment
  float offset_spacing; //spacing between neighboring raster strokes
  float height_offset; //TCP z-offset from surface
  float min_hole_size; //holes larger than this will split a segment
  float min_segment_size; //small path segments will be dropped
};

struct CloudConfig
{
  int rows;
  int cols;
  int sample_rate;
  int scaling_factor;
};

class HeatSurfacePlanner
{
public:
  HeatSurfacePlanner()
  {
    config_ = getDefaultConfig();
    init();
  }
  HeatSurfacePlanner(ProcessConfig& config)
  {
    config_.run_density = config.run_density;
    config_.offset_spacing = config.offset_spacing;
    config_.height_offset = config.height_offset;
    config_.min_hole_size = config.min_hole_size;
    config_.min_segment_size = config.min_segment_size;
    init();
  }

  void init()
  {
    // standard parameters
    nSourceSets_ = 0;
    smoothness_ = -1.0; //TODO is this necessary?
    boundaryConditions_ = -1.0; //TODO is this neceessary?
    verbose_ = 0; //TODO is this necessary?

    /* initialize data structures*/
//    hmContextInitialize(&context_);
//    hmTriMeshInitialize(&surface_);
//    hmTriDistanceInitialize(&distance_);
  }

  static ProcessConfig getDefaultConfig()
  {
    ProcessConfig C;
    C.run_density = 0.1;
    C.offset_spacing = 0.2;
    C.height_offset = 0.0;
    C.min_hole_size = 0.2;
    C.min_segment_size = 0.5;
//    C.raster_rot_offset = 0.0;
    return (C);
  }

  static CloudConfig getDefaultCloudConfig(){
    CloudConfig cloud_config;
    cloud_config.rows = 1544;
    cloud_config.cols = 2064;
    cloud_config.sample_rate = 4;
    cloud_config.scaling_factor = 1000.0;
    return cloud_config;
  }


  ~HeatSurfacePlanner() {}

  void planPaths(const sensor_msgs::PointCloud2& cloud,
                 std::vector<geometry_msgs::PoseArray>& paths,
                 heat::CloudConfig& cloud_config);

  void convertMesh(const sensor_msgs::PointCloud2& cloud,
                   heat::CloudConfig& cloud_config);

  /** @brief from a sequence of points, create a sequence of poses(path).
   *  aligns z with local mesh normal,
   *  aligns x toward next point in sequence
   *  aligns y using right hand rule with
   **/
//  bool createPoseArray(const std::vector<int>& path_indices, geometry_msgs::PoseArray& poses);

  /** @brief find the center of mass of a triangle and its area
   *  @input mesh the mesh containing vertices and triangles
   *  @input id   the index of the triangle for consideration
   *  @output center The center of mass of this triangle
   *  @output area   The area of this triangle
   **/
//  bool getCellCentroidData(std::shared_ptr<geometrycentral::surface::SurfaceMesh> mesh,
//                           std::shared_ptr<geometrycentral::surface::VertexPositionGeometry> geometry,
//                           const int id, Eigen::Vector3d& center, double& area);

  /** @brief find the cutting plane given the mesh and its raster angle
   *  @input mesh the mesh containing vertices and triangles
   *  @input raster_angle angle to rotate around mid-axis of principal component
   *  @output N the normal vector for the plane
   *  @output D the right hand side of the plane equation N_x X + N_y Y + N_z Z = D
   **/
//  bool getCuttingPlane(std::shared_ptr<geometrycentral::surface::SurfaceMesh> mesh,
//                       std::shared_ptr<geometrycentral::surface::VertexPositionGeometry> geometry,
//                       const double raster_angle, Eigen::Vector3d& N, double& D);

//  double estimateMeshTime(const shape_msgs::Mesh& mesh);

//  void subtractVector(geometry_msgs::Point &out, geometry_msgs::Point &in_1, geometry_msgs::Point &in_2);

//  void normVector(double &norm, geometry_msgs::Point &vec);


public:
  ProcessPath path_;
  ProcessConfig config_;

//  hmContext context_;
//  hmTriMesh surface_;
//  hmTriDistance distance_;

  std::shared_ptr<geometrycentral::surface::SurfaceMesh> mesh_;
  std::shared_ptr<geometrycentral::surface::VertexPositionGeometry> geometry_;


  // parameters
  int nSourceSets_;
  double smoothness_;
  double boundaryConditions_;
  char verbose_ = 0;
  double time_coeff = 1;

};  // end class HeatSurfacePlanner

}  // end namespace heat
#endif  // INCLUDE_HEAT_SURFACE_PLANNER_H
