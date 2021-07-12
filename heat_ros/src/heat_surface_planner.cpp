#include <stdio.h>
#include <ros/ros.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <heat_ros/heat_surface_planner.hpp>
#include <heat_ros/hm_heat_path.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include "pcl_ros/point_cloud.h"

#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/meshio.h"
#include "geometrycentral/surface/surface_mesh.h"
#include "geometrycentral/utilities/vector3.h"
#include "geometrycentral/surface/heat_method_distance.h"
#include <heat_ros/heat_surface_planner.hpp>
#include <geometrycentral/surface/simple_polygon_mesh.h>
#include "geometrycentral/surface/surface_mesh_factories.h"


#define DEBUG_CUT_AXIS 1
namespace heat
{
void HeatSurfacePlanner::planPaths(const sensor_msgs::PointCloud2& cloud,
                                   std::vector<geometry_msgs::PoseArray>& paths,
                                   heat::CloudConfig& cloud_config)
{

  bool use_hardcoded_sources = true;
  std::vector<int> source_indices;
//  std::tie(mesh_, geometry_) = geometrycentral::surface::loadMesh("/home/cwolfe/heat_method_ws/src/Part Meshes/meshes_from_clouds/subsampled_mesh_06-08.ply"); //03.02
  convertMesh(cloud, cloud_config);
  mesh_->printStatistics();
  int nv = mesh_->nVertices();
  geometrycentral::surface::VertexData<double> is_source;
  if (!use_hardcoded_sources)
  {
    ROS_INFO("Generate using edge detection. not implemented yet");
    //TODO IMPLEMENT EDGE DETECTION
  }
  else
  {                                                        // use provided sources
    ROS_INFO("Use harcoded sources");

    source_indices = {38478, 38477, 38476, 38475, 38837, 39198, 39197, 39196, 39195, 39554,
                      39553, 39552, 39910, 40265, 40264, 40263, 40262, 40261, 40260, 40259,
                      40258, 40257, 40256, 40255, 40254, 40253, 40252, 40251, 40250, 40249,
                      40248, 40247, 40246, 40245, 40244, 40243, 40242, 40241, 40592, 40591,
                      40590, 40589, 40588, 40587, 40935, 40934, 40933, 40932, 41276, 41275,
                      41274, 41273, 41272, 41271, 41270, 41609, 41608, 41940, 41939, 41938,
                      41937, 41936, 41935, 41934, 42261, 42260, 42580, 42895, 42894, 42893,
                      43199, 43198, 43197, 43196, 43195, 43194, 43193, 43192, 43191, 43190,
                      43490, 43489, 43781, 43780, 43779, 44062, 44060, 44060, 44059, 44058,
                      44057, 44056, 44055, 44054, 44053, 44052, 44051, 44318, 44317, 44316,
                      44315, 44566, 44564, 44562, 44560, 44559, 44793, 44791, 44789, 44788,
                      44786, 44784, 44782, 44780, 44779, 44778, 44777, 44776, 44775, 44774,
                      44983, 44982, 44980, 44978, 44977, 44975, 44974, 44972, 44970, 44968,
                      44966, 44965, 44964, 44963, 44962, 44961, 44960, 44959, 44958, 44957,
                      44956, 44955, 44954, 44953, 44952, 44951, 44950, 44949, 44948, 44947,
                      44946, 44945, 44944, 44943, 44942, 44941, 44940, 44939, 44938, 44937,
                      44936, 44935, 44934, 44933, 44932, 44931, 44930, 44929, 44928, 44927,
                      44926, 44925, 44924, 44923, 44922, 44921, 44921, 44920, 44919, 44918,
                      44917, 44916, 44915, 44914, 44913, 44912, 44911, 44910, 44909, 44908,
                      44907, 44906, 44905, 44904, 44903, 44902, 44901, 44900, 44899, 44898,
                      44897, 44896, 44895, 44894, 44893, 44892, 44891, 44890, 44889, 44888,
                      44887, 44886, 44885, 44884, 44883, 44882, 44881, 44880, 44879, 44878,
                      44877, 44876, 44875, 44874, 44873, 44872, 44661, 44660, 44659, 44658,
                      44657, 44656, 44655, 44654, 44653, 44652, 44651, 44650, 44649, 44648,
                      44647, 44646, 44645, 44644, 44643, 44642, 44641, 44640, 44639, 44638,
                      44637, 44636, 44635, 44634, 44633, 44632, 44631, 44630, 44629, 44628,
                      44627, 44626, 44625, 44624, 44623, 44622, 44621, 44620, 44619, 44618,
                      44617, 44616, 44615, 44614, 44613, 44612, 44611, 44610, 44609, 44608,
                      44607, 44606, 44605, 44604, 44603, 44602, 44601, 44600, 44599, 44598,
                      44597, 44596, 44595, 44594, 44593, 44593, 44592, 44591, 44590, 44589,
                      44587, 44586, 44584, 44583, 44582, 44581, 44580, 44579, 44577, 44575,
                      44573, 44571, 44570, 44334, 44332, 44331, 44329, 44328, 44326, 44073,
                      44071, 44069, 44068, 43799, 43797, 43796, 43794, 43793, 43792, 43507};
  }
  std::cout << "num source indices = " << source_indices.size();
  std::cout << "local_source_indices \n";
  for (auto i = source_indices.begin(); i != source_indices.end(); ++i)
      std::cout << *i << ' ';

  ROS_INFO("call heat solver");
  geometrycentral::surface::HeatMethodDistanceSolver heat_solver(*geometry_, time_coeff); //TODO can heat_solver be declared in the header?
  ROS_INFO("end call heat solver");
  std::vector<geometrycentral::surface::Vertex> source_verts;
  for (int i=0; i<source_indices.size(); ++i){
    source_verts.push_back(mesh_->vertex(source_indices[i]));
  }
  ROS_INFO("Transfer local sources to sources");

  geometrycentral::surface::VertexData<double> dist_to_source;
  dist_to_source = heat_solver.computeDistance(source_verts); //TODO seems to return wrong distance values
//  Eigen::Matrix<double, -1, 1> dist_to_source_vec = dist_to_source.toVector();
  double max_dist_val = -999999999.9;
  for (int i=0; i < dist_to_source.size(); i++){
    if (dist_to_source[i] > max_dist_val){
      max_dist_val = dist_to_source[i];
    }
  }
  ROS_INFO_STREAM("MAX DIST VAL = " << max_dist_val);

  double time = heat_solver.getTime(); //TODO If change multiplier of time in gc, need to change here too 06.25
  hmTriHeatPaths THP(mesh_, geometry_, source_indices, dist_to_source, config_.offset_spacing, time);
  THP.compute(source_indices);

  for (int i = 0; i < (int)THP.pose_arrays_.size(); i++)
  {
    geometry_msgs::PoseArray PA;
    for (int j = 0; j < (int)THP.pose_arrays_[i].size(); j++)
    {
      geometry_msgs::Pose P;
      P.position.x = THP.pose_arrays_[i][j].x;
      P.position.y = THP.pose_arrays_[i][j].y;
      P.position.z = THP.pose_arrays_[i][j].z;
      P.orientation.w = THP.pose_arrays_[i][j].qw;
      P.orientation.x = THP.pose_arrays_[i][j].qx;
      P.orientation.y = THP.pose_arrays_[i][j].qy;
      P.orientation.z = THP.pose_arrays_[i][j].qz;
      PA.poses.push_back(P);
    }
    paths.push_back(PA);
  }

  //TODO Return bool to indicate if succeeded or not
}  // end of plan_paths()


void HeatSurfacePlanner::convertMesh(const sensor_msgs::PointCloud2& cloud,
                                     heat::CloudConfig& cloud_config){
  pcl::PointCloud<pcl::PointXYZ> cloud_;
  pcl::fromROSMsg(cloud, cloud_);
  Eigen::MatrixXd vMat(cloud_.size(), 3);
  Eigen::MatrixXi fMat(cloud_.size()*2, 3);
  std::vector<int> ind_map;
  ROS_INFO_STREAM("cloud size = " << cloud_.size());

  int reduced_rows = cloud_config.rows/cloud_config.sample_rate;
  int reduced_cols = cloud_config.cols/cloud_config.sample_rate;

  int index;
  int num_valid_pts = 0;
  for (int i=0; i<reduced_rows; i++){
    for (int j=0; j<reduced_cols; j++){
      index = i*cloud_config.cols*cloud_config.sample_rate + j*cloud_config.sample_rate;
      pcl::PointXYZ pt = cloud_.at(index);
      //      ROS_INFO_STREAM("x=" << pt.x <<", y="<<pt.y<<", z="<<pt.z);
      if (pt.x!=0 && pt.y!=0 && pt.z!=0){
        vMat(num_valid_pts, 0) = pt.x/cloud_config.scaling_factor;
        vMat(num_valid_pts, 1) = pt.y/cloud_config.scaling_factor;
        vMat(num_valid_pts, 2) = pt.z/cloud_config.scaling_factor;
        ind_map.push_back(num_valid_pts);
        num_valid_pts ++;
      }
      else{
        ind_map.push_back(-1);
      }
    }
  }

  ROS_INFO_STREAM("FINISH VERTICES. Total num = " << num_valid_pts);
  int num_triangles = 0;
  int num_discarded_triangles = 0;
  for (int i=0; i<reduced_rows-1; i++){
    for (int j=0; j<reduced_cols-1; j++){
      int ind_1 = i*reduced_cols + j;
      int ind_2 = (i+1)*reduced_cols + j;
      int ind_3 = i*reduced_cols + j+1;
      if ((ind_map.at(ind_1) != -1) && (ind_map.at(ind_2) != -1) && (ind_map.at(ind_3) != -1)){
        fMat(num_triangles, 0) = ind_map.at(ind_1);
        fMat(num_triangles, 1) = ind_map.at(ind_2);
        fMat(num_triangles, 2) = ind_map.at(ind_3);
        num_triangles++;
      }
      else {
        num_discarded_triangles++;
      }

      int ind_4 = i*reduced_cols + j+1;
      int ind_5 = (i+1)*reduced_cols + j;
      int ind_6 = (i+1)*reduced_cols + j+1;
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

  ROS_INFO("CREATE MESH AND GEOMETRY");
  std::tie(mesh_, geometry_) = geometrycentral::surface::makeSurfaceMeshAndGeometry(reduced_vMat, reduced_fMat);

  //TODO return bool to indicate if succeeded or not
}


/*
double HeatSurfacePlanner::estimateMeshTime(const shape_msgs::Mesh& mesh){
  size_t nFaces = mesh.triangles.size();;
//  size_t* facesBegin = distance->surface->faces;
//  size_t* facesEnd = facesBegin + 3 * nFaces;
//  int f[3];
//  geometry_msgs/Point* vertices = mesh.vertices;
  geometry_msgs::Point p0, p1, p2;
  geometry_msgs::Point e01, e12, e20;
  double e01_norm, e12_norm, e20_norm;
  double meanEdgeLength = 0.;
  double nEdges = 0.;

  // iterate over faces
  int count = 0;
  for (auto f = mesh.triangles.begin(); f != mesh.triangles.end(); ++f)
  {
    count +=1;
    / get vertex coordinates p0, p1, p2
//    ROS_ERROR_STREAM("face vertex 0: " << f->vertex_indices.at(0));
//    ROS_ERROR_STREAM("face vertex 1: " << f->vertex_indices.at(1));
//    ROS_ERROR_STREAM("face vertex 2: " << f->vertex_indices.at(2));
//    ROS_ERROR_STREAM("vertex_indices size = " << f->vertex_indices.size());
    p0 = mesh.vertices[f->vertex_indices.at(0)];
    p1 = mesh.vertices[f->vertex_indices.at(1)];
    p2 = mesh.vertices[f->vertex_indices.at(2)];
//    ROS_ERROR_STREAM("face vertex 0: " << p0);
//    ROS_ERROR_STREAM("face vertex 1: " << p1);
//    ROS_ERROR_STREAM("face vertex 2: " << p2);
    //add edge lengths to mean
    subtractVector(e01, p1, p0);
    subtractVector(e12, p2, p1);
    subtractVector(e20, p0, p2);
    normVector(e01_norm, e01);
    normVector(e12_norm, e12);
    normVector(e20_norm, e20);

    meanEdgeLength += e01_norm + e12_norm + e20_norm;
    nEdges += 3.;
  }
  ROS_ERROR_STREAM("count = " << count);
  meanEdgeLength /= nEdges;

  // set t to square of mean edge length
  double time = meanEdgeLength * meanEdgeLength;
  ROS_ERROR_STREAM("FINAL TIME = " << time);
  return time;
}

void HeatSurfacePlanner::subtractVector(geometry_msgs::Point &out, geometry_msgs::Point &in_1, geometry_msgs::Point &in_2){
  out.x = in_1.x - in_2.x;
  out.y = in_1.y - in_2.y;
  out.z = in_1.z - in_2.z;
}

void HeatSurfacePlanner::normVector(double &norm, geometry_msgs::Point &vec){
  norm = sqrt((vec.x*vec.x) + (vec.y*vec.y) + (vec.z*vec.z));
}

bool HeatSurfacePlanner::getCellCentroidData(std::shared_ptr<geometrycentral::surface::SurfaceMesh> mesh,
                                             std::shared_ptr<geometrycentral::surface::VertexPositionGeometry> geometry,
                                             const int id,
                                             Eigen::Vector3d& center,
                                             double& area)
{
  geometrycentral::surface::Face face = mesh->face(id);
  std::vector<geometrycentral::Vector3> adj_verts;
  for(geometrycentral::surface::Vertex v : face.adjacentVertices()) {
    geometrycentral::Vector3 vec = geometry_->inputVertexPositions[v];
    adj_verts.push_back(vec);
  }
  geometrycentral::Vector3 pt1 = adj_verts.at(0);
  geometrycentral::Vector3 pt2 = adj_verts.at(1);
  geometrycentral::Vector3 pt3 = adj_verts.at(2);
  Eigen::Vector3d va(pt1.x - pt2.x, pt1.y - pt2.y, pt1.z - pt2.z);
  Eigen::Vector3d vb(pt2.x - pt3.x, pt2.y - pt3.y, pt2.z - pt3.z);
  Eigen::Vector3d vc(pt3.x - pt1.x, pt3.y - pt1.y, pt3.z - pt1.z);
  double a = va.norm();
  double b = vb.norm();
  double c = vc.norm();
  double s = (a + b + c) / 2.0;
  area = sqrt(s * (s - a) * (s - b) * (s - c));
  center.x() = (pt1.x + pt2.x + pt3.x) / 3.0;
  center.y() = (pt1.y + pt2.y + pt3.y) / 3.0;
  center.z() = (pt1.z + pt2.z + pt3.z) / 3.0;
  return (true);
}  // end of getCellCentroidData()


bool HeatSurfacePlanner::getCuttingPlane(std::shared_ptr<geometrycentral::surface::SurfaceMesh> mesh,
                                         std::shared_ptr<geometrycentral::surface::VertexPositionGeometry> geometry,
                                         const double raster_angle,
                                         Eigen::Vector3d& N,
                                         double& D)
{
  // TODO put this in its own function
  // find center of mass of mesh
  Eigen::Vector3d C;
  double A;
  std::vector<Eigen::Vector3d> centers;
  std::vector<double> areas;
  for (int i = 0; i < mesh->nFaces(); i++)
  {
    Eigen::Vector3d Ci;
    double Ai;
    getCellCentroidData(mesh, geometry, i, Ci, Ai);
    C += Ci * Ai;
    A += Ai;
    centers.push_back(Ci);
    areas.push_back(Ai);
  }
  C = C / A;
  Eigen::Matrix<double, 3, 3> Inertia;
  for (int i = 0; i < mesh->nFaces(); i++)
  {
    double xk = centers[i].x() - C.x();
    double yk = centers[i].y() - C.y();
    double zk = centers[i].z() - C.z();
    Inertia(0, 0) += areas[i] * (yk * yk + zk * zk);
    Inertia(1, 1) += areas[i] * (xk * xk + zk * zk);
    Inertia(2, 2) += areas[i] * (xk * xk + yk * yk);
    Inertia(0, 1) -= areas[i] * xk * yk;
    Inertia(0, 2) -= areas[i] * xk * zk;
    Inertia(1, 2) -= areas[i] * yk * zk;
  }
  Inertia(1, 0) = Inertia(0, 1);
  Inertia(2, 0) = Inertia(0, 2);
  Inertia(2, 1) = Inertia(2, 1);
  Eigen::JacobiSVD<Eigen::Matrix3d> SVD(Inertia, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::MatrixXd U = SVD.matrixU();

  // These two axes and the center defines the plane
  Eigen::Vector3d max_axis(U(0, 0), U(1, 0), U(2, 0));
  Eigen::Vector3d mid_axis(U(0, 1), U(1, 1), U(2, 1));
  Eigen::Vector3d min_axis(U(0, 2), U(1, 2), U(2, 2));
  if (DEBUG_CUT_AXIS)
  {
    printf("Max_axis %lf %lf %lf\n", max_axis[0], max_axis[1], max_axis[2]);
    printf("Mid_axis %lf %lf %lf\n", mid_axis[0], mid_axis[1], mid_axis[2]);
    printf("Min_axis %lf %lf %lf\n", min_axis[0], min_axis[1], min_axis[2]);
    printf("raster angle = %lf\n", raster_angle);
  }
  Eigen::Quaterniond rot(Eigen::AngleAxisd(raster_angle, min_axis));
  rot.normalize();

  N = rot.toRotationMatrix() * mid_axis;

  // equation of a plane through a center point with a given normal is
  // nx(X-cx) + ny(Y-cy) + nz(Z-cz) =0
  // nxX + nyY + nzZ = nxcx + nycy + nzcz = d where d = nxcx + nycy + nzcz
  // Distance from point to plan D = nxX + nyY + nzZ -d
  D = N.x() * C.x() + N.y() * C.y() + N.z() * C.z();
  if (DEBUG_CUT_AXIS)
    printf("Plane equation %6.3lfx %6.3lfy %6.3lfz = %6.3lf\n", N[0], N[1], N[2], D);
  return (true);
}
*/


}  // end of namespace heat
