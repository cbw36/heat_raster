/**
 * @file smooth_pose_traj.h
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

#ifndef POLYLINE_SMOOTHER_HPP
#define POLYLINE_SMOOTHER_HPP

#include <stdio.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <boost/math/interpolators/cubic_b_spline.hpp>
#include <eigen3/Eigen/Eigen>

#include <shape_msgs/Mesh.h>


namespace PolylineSmoother
{

  struct Pose{
    double qx,qy,qz,qw,x,y,z;
  };

  class PolylineSmoother
  {

  public:
    PolylineSmoother()
      {
      }
    PolylineSmoother(const double& pt_spacing, const shape_msgs::Mesh& mesh,
                     const std::vector<int> source_indices);

    ~PolylineSmoother()
    {}

    void face_normal(int vertex_index, Eigen::Vector3d& normal_vec);

    void planPaths(const shape_msgs::Mesh& mesh,
                   const std::vector<int>& source_indices,
                   std::vector<geometry_msgs::PoseArray>& paths);

    void compute_pose_arrays();

  public:
    double pt_spacing_, total_distance_, max_t_;
    std::vector<int> source_indices_;
    const shape_msgs::Mesh mesh_;
//    std::vector<Pose> pose_arrays_;
    geometry_msgs::PoseArray pose_arrays_;

  private:


    /** \brief Number of vertices. */
    size_t nVertices;

    /** \brief Vertex coordinates as consecutive (x,y,z) triples. [3 x nVertices] */
    double* vertices;

    /** \brief Texture coordinates as consecutive (u,v) pairs. [2 x nVertices] */
    double* texCoords;

    /** \brief Number of triangles. */
    size_t nFaces;

    /** \brief Triangles as consecutive (i,j,k) triples of 0-based vertex indices. [3 x nFaces] */
    size_t* faces;


  }; // end class PolylineSmoother

} // end namespace PolylineSmoother


#endif // POLYLINE_SMOOTHER_HPP
