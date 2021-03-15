#include <stdio.h>
#include <ros/ros.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <heat_ros/heat_surface_planner.hpp>
#include <heat_ros/hm_heat_path.hpp>

#define DEBUG_CUT_AXIS 1
namespace heat
{
void HeatSurfacePlanner::planPaths(const shape_msgs::Mesh& mesh,
                                   const std::vector<int>& source_indices,
                                   std::vector<geometry_msgs::PoseArray>& paths)
{
  std::vector<int> local_source_indices;
  for (int i = 0; i < (int)source_indices.size(); i++)
    local_source_indices.push_back(source_indices[i]);

  std::cout << "local_source_indices \n";
  for (auto i = local_source_indices.begin(); i != local_source_indices.end(); ++i)
      std::cout << *i << ' ';
  std::cout << "\n source_indices \n";
  for (auto i = source_indices.begin(); i != source_indices.end(); ++i)
      std::cout << *i << ' ';


  std::tie(mesh_, geometry_) = geometrycentral::surface::loadMesh("/home/cwolfe/grinding_blades_ws/src/Part Meshes/bunny.obj"); //03.02

  /*
//    hmTriMeshInitialize(&surface_); //02.23

  // convert Mesh to hmTriMesh //03.02
//  surface_.nVertices = mesh.vertices.size(); //03.02
//  surface_.nFaces = mesh.triangles.size(); //03.02

  // allocate
//  surface_.vertices = (double*)malloc(surface_.nVertices * 3 * sizeof(double)); //03.02
//  surface_.texCoords = (double*)malloc(surface_.nVertices * 2 * sizeof(double)); //03.02
//  surface_.faces = (size_t*)malloc(surface_.nFaces * 3 * sizeof(size_t));

  // copy vertices and faces
//  double* v = surface_.vertices; //03.02
//  for (int i = 0; i < surface_.nVertices; i++)  // copy each vertex //03.02
//  { //03.02
//    v[0] = mesh.vertices[i].x; //03.02
//    v[1] = mesh.vertices[i].y; //03.02
//    v[2] = mesh.vertices[i].z; //03.02
//    v += 3; //03.02
//  }  // end copy each vertex //03.02

//  size_t* f = surface_.faces; //03.02
//  for (int i = 0; i < surface_.nFaces; i++)  // copy each triangle //03.02
//  { //03.02
//    f[0] = mesh.triangles[i].vertex_indices[0]; //03.02
//    f[1] = mesh.triangles[i].vertex_indices[1]; //03.02
//    f[2] = mesh.triangles[i].vertex_indices[2]; //03.02
//    f += 3; //03.02
//  }  // end copy each triangle //03.02
//  distance_.surface = &surface_; //03.02

//   set time for heat flow  //TODO WHAT IS THIS? 03.02
//  hmTriDistanceEstimateTime(&distance_); //03.02

//  if (smoothness_ > 0.0) //03.02
//  { //03.02
//    distance_.time *= smoothness_; //03.02
//  } //03.02

//   specify boundary conditions  //03.02
//  if (boundaryConditions_ > 0.) //03.02
//  { //03.02
//    hmTriDistanceSetBoundaryConditions(&distance_, boundaryConditions_); //03.02
//  } //03.02

//   specify verbosity
//  distance_.verbose = verbose_; //03.02

//   compute distance
//  hmTriDistanceBuild(&distance_); //03.02
  */

//  mesh_->printStatistics();
  int nv = mesh_->nVertices();
  geometrycentral::surface::VertexData<double> is_source;
  if (local_source_indices.size() == 0)
  {
    Eigen::Vector3d N;
    double D;
    std::cout << "config_.raster_rot_offset = " << config_.raster_rot_offset << "\n";
    getCuttingPlane(mesh, config_.raster_rot_offset, N, D);

    // Create a new sources vector that contains all points within a small distance from this plane
    int num_source_verts = 0;
    for (int i = 0; i < (int)mesh.vertices.size(); i++)
    {
      double d = N.x() * mesh.vertices[i].x + N.y() * mesh.vertices[i].y + N.z() * mesh.vertices[i].z - D;
      if (fabs(d) < config_.raster_spacing / 7.0)
      {
        num_source_verts++;
//        is_source[i] = 1.0;
        local_source_indices.push_back(i);
      }
    }
    if (DEBUG_CUT_AXIS)
      printf("found %d source vertices\n", num_source_verts);
  }
  else
  {                                                        // use provided sources
    for (int i = 0; i < local_source_indices.size(); i++)  // set sources
    {
      int n = local_source_indices[i];
      if (n >= nv)
      {
        printf("Source index %d is set to %d but we only have %d vertices in mesh\n", i, n, nv);
      }
      else
      {
//        is_source[n] = 1.;
      }
    }  // end setting sources
  }

  std::cout << "num source indices = " << local_source_indices.size();
  std::cout << "local_source_indices \n";
  for (auto i = local_source_indices.begin(); i != local_source_indices.end(); ++i)
      std::cout << *i << ' ';


  geometrycentral::surface::HeatMethodDistanceSolver heat_solver(*geometry_); //TODO can this go in the header?
// calculate the distances
//  hmTriDistanceUpdate(&mesh_);
  std::vector<geometrycentral::surface::Vertex> source_verts;
  for (int i=0; i<local_source_indices.size(); ++i){
    source_verts.push_back(mesh_->vertex(local_source_indices[i]));
  }

  geometrycentral::surface::VertexData<double> dist_to_source;
  dist_to_source = heat_solver.computeDistance(source_verts); //TODO seems to return wrong distance values
  Eigen::Matrix<double, -1, 1> dist_to_source_vec = dist_to_source.toVector();
//  ROS_INFO_STREAM("size = " << dist_to_source_vec.size() << ", cols = "<< dist_to_source_vec.cols() <<
//                  ", rows = " << dist_to_source_vec.rows() << ", x=" << dist_to_source_vec.x() <<
//                  ", y=" << dist_to_source_vec.y() << ", z=" << dist_to_source_vec.z() <<
//                  ", w=" << dist_to_source_vec.w());
//  ROS_INFO_STREAM("element 0: " << dist_to_source_vec[0]);

  ROS_INFO_STREAM("sources size = " << source_verts.size());
  ROS_INFO_STREAM("num rows = "<< dist_to_source.size());

  double time = estimateMeshTime(mesh);
  hmTriHeatPaths THP(mesh_, config_.raster_spacing, dist_to_source, time);
  THP.compute(mesh_, local_source_indices);

  ROS_INFO_STREAM("nedges = " << mesh_->nEdges() << ", halfedges = " << mesh_->nHalfedges());

/*
//  for (int i = 0; i < (int)THP.pose_arrays_.size(); i++)
//  {
//    geometry_msgs::PoseArray PA;
//    for (int j = 0; j < (int)THP.pose_arrays_[i].size(); j++)
//    {
//      geometry_msgs::Pose P;
//      P.position.x = THP.pose_arrays_[i][j].x;
//      P.position.y = THP.pose_arrays_[i][j].y;
//      P.position.z = THP.pose_arrays_[i][j].z;
//      P.orientation.w = THP.pose_arrays_[i][j].qw;
//      P.orientation.x = THP.pose_arrays_[i][j].qx;
//      P.orientation.y = THP.pose_arrays_[i][j].qy;
//      P.orientation.z = THP.pose_arrays_[i][j].qz;
//      PA.poses.push_back(P);
//    }
//    paths.push_back(PA);
//  }

  // deallocate data structures
  // TODO there are known memory leaks fix them
//  hmTriMeshDestroy(&surface_);
//  hmTriDistanceDestroy(&distance_);
//  hmContextDestroy(&context_);
*/

}  // end of plan_paths()

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

  /* iterate over faces */
  int count = 0;
  for (auto f = mesh.triangles.begin(); f != mesh.triangles.end(); ++f)
  {
    count +=1;
    /* get vertex coordinates p0, p1, p2 */
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
    /* add edge lengths to mean */
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

  /* set t to square of mean edge length */
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

bool HeatSurfacePlanner::getCellCentroidData(const shape_msgs::Mesh& mesh,
                                             const int id,
                                             Eigen::Vector3d& center,
                                             double& area)
{
  geometry_msgs::Point pt1 = mesh.vertices[(mesh.triangles[id].vertex_indices[0])];
  geometry_msgs::Point pt2 = mesh.vertices[(mesh.triangles[id].vertex_indices[1])];
  geometry_msgs::Point pt3 = mesh.vertices[(mesh.triangles[id].vertex_indices[2])];
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

bool HeatSurfacePlanner::getCuttingPlane(const shape_msgs::Mesh& mesh,
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
  for (int i = 0; i < mesh.triangles.size(); i++)
  {
    Eigen::Vector3d Ci;
    double Ai;
    getCellCentroidData(mesh, i, Ci, Ai);
    C += Ci * Ai;
    A += Ai;
    centers.push_back(Ci);
    areas.push_back(Ai);
  }
  C = C / A;
  Eigen::Matrix<double, 3, 3> Inertia;
  for (int i = 0; i < mesh.triangles.size(); i++)
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
}  // end of namespace heat
