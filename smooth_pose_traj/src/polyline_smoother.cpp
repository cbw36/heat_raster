#include <ros/ros.h>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <smooth_pose_traj/polyline_smoother.hpp>
#include <eigen3/Eigen/Eigen>

#define DEBUG_QUAT 0 // computaton of quaternion of poses
#define DEBUG_NORMALS 0 // computation of normals


namespace PolylineSmoother
{

  PolylineSmoother::PolylineSmoother(const double& pt_spacing, const shape_msgs::Mesh& mesh,
                                     const std::vector<int> source_indices)
    : pt_spacing_(pt_spacing), mesh_(mesh), source_indices_(source_indices)
  {}// end constructor for PolylineSmoother


  void PolylineSmoother::face_normal(int vertex_index, Eigen::Vector3d& normal_vec)
  {
    // find all faces containing this vertex
    std::vector<size_t> included_faces;
    for(size_t i=0; i<mesh_.triangles.size(); i++)
    {
      shape_msgs::MeshTriangle triangle = mesh_.triangles.at(i);
      if(triangle.vertex_indices[0] == vertex_index || triangle.vertex_indices[1] == vertex_index || triangle.vertex_indices[2] == vertex_index)
      {
        included_faces.push_back(i);
      }
    }

    // find average normal of all faces
    normal_vec(0) = 0.0;
    normal_vec(1) = 0.0;
    normal_vec(2) = 0.0;
    for(int i=0;i<(int)included_faces.size();i++)
    {
      shape_msgs::MeshTriangle triangle = mesh_.triangles.at(included_faces[i]);
      Eigen::Vector3d pt1(mesh_.vertices[triangle.vertex_indices[0]].x,
                          mesh_.vertices[triangle.vertex_indices[0]].y,
                          mesh_.vertices[triangle.vertex_indices[0]].z);
      Eigen::Vector3d pt2(mesh_.vertices[triangle.vertex_indices[1]].x,
                          mesh_.vertices[triangle.vertex_indices[1]].y,
                          mesh_.vertices[triangle.vertex_indices[1]].z);
      Eigen::Vector3d pt3(mesh_.vertices[triangle.vertex_indices[2]].x,
                          mesh_.vertices[triangle.vertex_indices[2]].y,
                          mesh_.vertices[triangle.vertex_indices[2]].z);
      Eigen::Vector3d v1 = pt2-pt1;
      Eigen::Vector3d v2 = pt3-pt2;
      Eigen::Vector3d nv = v1.cross(v2);
      nv.normalize();
      if(DEBUG_NORMALS)
      {
        printf("pt1 = [%6.3lf %6.3lf %6.3lf];\n",pt1(0),pt1(1),pt1(2));
        printf("pt2 = [%6.3lf %6.3lf %6.3lf];\n",pt2(0),pt2(1),pt2(2));
        printf("pt3 = [%6.3lf %6.3lf %6.3lf];\n",pt3(0),pt3(1),pt3(2));
        printf("nv  = [%6.3lf %6.3lf %6.3lf];\n",nv(0),nv(1),nv(2));
      }
      normal_vec = normal_vec + nv;
    }
    normal_vec.normalize(); // re-normalize should be same as divide by number of normals
  }


  void PolylineSmoother::compute_pose_arrays()
  {
    // create a set of poses from the sequence by:
    // 1. Use surface normal for z_vec of pose
    // 2. Use direction to next in sequence as x_vec
    // 3. Use z_vec cross x_vec as y
    // 4. compute quaternion from 3x3 rotation defined by x_vec|y_vec|z_vec
    // 5. use most recent quat for last in sequence

    geometry_msgs::PoseArray new_pose_array;
    for(int i=0; i<source_indices_.size()-1; i++)
    {
      geometry_msgs::Pose P;
      Eigen::Vector3d z_vec;
      double zv[3];

      int vj = source_indices_.at(i);
      int vn = source_indices_.at(i+1); //TODO what to do about i+1?
      face_normal(vj, z_vec); // most reliable
      if(DEBUG_QUAT) printf("zv = [ %6.3lf %6.3lf %6.3lf]\n", z_vec(0), z_vec(1), z_vec(2));
      Eigen::Vector3d p1(mesh_.vertices[vj].x, mesh_.vertices[vj].y, mesh_.vertices[vj].z);
      Eigen::Vector3d p2(mesh_.vertices[vn].x, mesh_.vertices[vn].y, mesh_.vertices[vn].z);
      Eigen::Vector3d x_vec = p2-p1;
      x_vec = x_vec - x_vec.dot(z_vec)*z_vec;
      if(x_vec.norm() != 0.0)
      {
        x_vec.normalize();
        Eigen::Vector3d y_vec = z_vec.cross(x_vec);
        y_vec.normalize();

        // the point is the vertex
        P.position.x = p1[0];
        P.position.y = p1[1];
        P.position.z = p1[2];
        Eigen::Matrix3d R(3,3);
        R(0,0) = x_vec(0); R(0,1) = y_vec(0); R(0,2) = z_vec(0);
        R(1,0) = x_vec(1); R(1,1) = y_vec(1); R(1,2) = z_vec(1);
        R(2,0) = x_vec(2); R(2,1) = y_vec(2); R(2,2) = z_vec(2);
        Eigen::Quaterniond Q(R);
        if(DEBUG_QUAT)
        {
           printf("Q:%6.3lf %6.3lf %6.3lf %6.3lf\n", Q.w(), Q.x(), Q.y(), Q.z());
           printf("R = [%6.3lf %6.3lf %6.3lf; \n     %6.3lf %6.3lf %6.3lf; \n     %6.3lf %6.3lf %6.3lf]\n",
           R(0,0),R(0,1),R(0,2),
           R(1,0),R(1,1),R(1,2),
           R(2,0),R(2,1),R(2,2));
        }
        P.orientation.w = Q.w();
        P.orientation.x = Q.x();
        P.orientation.y = Q.y();
        P.orientation.z = Q.z();
        new_pose_array.poses.push_back(P);
      }
      else
      {
        printf("normalize of x failed for vertex %d", i);
      }

    }// end for each vertex sequence
    pose_arrays_ = new_pose_array;
  }

}// end of namespace PolylineSmoother
