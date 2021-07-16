#include <stdio.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <heat_ros/heat_surface_planner.hpp>
#define DEBUG_CUT_AXIS 0
namespace heat
{
void HeatSurfacePlanner::planPaths(const shape_msgs::Mesh& mesh,
                                   const std::vector<int>& source_indices,
                                   std::vector<geometry_msgs::PoseArray>& paths)
{
//  int plane_size = 1000;
  std::vector<int> source_indices_hardcoded = {38478, 38477, 38476, 38475, 38837, 39198, 39197, 39196, 39195, 39554,
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

  std::vector<int> local_source_indices;
  for (int i = 0; i < (int)source_indices_hardcoded.size(); i++){
//  for (int i=0; i<plane_size; i++){
    local_source_indices.push_back(source_indices_hardcoded[i]);
//    local_source_indices.push_back(i);
  }

  hmTriMeshInitialize(&surface_);

  // convert Mesh to hmTriMesh
  surface_.nVertices = mesh.vertices.size();
  surface_.nFaces = mesh.triangles.size();

  // allocate
  surface_.vertices = (double*)malloc(surface_.nVertices * 3 * sizeof(double));
  surface_.texCoords = (double*)malloc(surface_.nVertices * 2 * sizeof(double));
  surface_.faces = (size_t*)malloc(surface_.nFaces * 3 * sizeof(size_t));

  // copy vertices and faces
  double* v = surface_.vertices;
  for (int i = 0; i < surface_.nVertices; i++)  // copy each vertex
  {
    v[0] = mesh.vertices[i].x;
    v[1] = mesh.vertices[i].y;
    v[2] = mesh.vertices[i].z;
    v += 3;
  }  // end copy each vertex

  size_t* f = surface_.faces;
  for (int i = 0; i < surface_.nFaces; i++)  // copy each triangle
  {
    f[0] = mesh.triangles[i].vertex_indices[0];
    f[1] = mesh.triangles[i].vertex_indices[1];
    f[2] = mesh.triangles[i].vertex_indices[2];
    f += 3;
  }  // end copy each triangle
  distance_.surface = &surface_;

  /* set time for heat flow */
  hmTriDistanceEstimateTime(&distance_);

  if (smoothness_ > 0.0)
  {
    distance_.time *= smoothness_;
  }

  /* specify boundary conditions */
  if (boundaryConditions_ > 0.)
  {
    hmTriDistanceSetBoundaryConditions(&distance_, boundaryConditions_);
  }

  /* specify verbosity */
  distance_.verbose = verbose_;

  /* compute distance */
  hmTriDistanceBuild(&distance_);

  int nv = distance_.surface->nVertices;
  hmClearArrayDouble(distance_.isSource.values, nv, 0.);
  if (local_source_indices.size() == 0)
  {
    int num_source_verts = 0;
    for (int i = 0; i < (int)mesh.vertices.size(); i++){
      if (mesh.vertices[i].y == 0){
        local_source_indices.push_back(i);
        num_source_verts++;
        distance_.isSource.values[i] = 1.0;
      }
    }
    printf("found %d source vertices\n", num_source_verts);
//    Eigen::Vector3d N;
//    double D;
//    getCuttingPlane(mesh, config_.raster_rot_offset, N, D);

//    // Create a new sources vector that contains all points within a small distance from this plane
//    int num_source_verts = 0;
//    for (int i = 0; i < (int)mesh.vertices.size(); i++)
//    {
//      double d = N.x() * mesh.vertices[i].x + N.y() * mesh.vertices[i].y + N.z() * mesh.vertices[i].z - D;
//      if (fabs(d) < config_.raster_spacing / 7.0)
//      {
//        num_source_verts++;
//        distance_.isSource.values[i] = 1.0;
//        local_source_indices.push_back(i);
//      }
//    }
//    if (DEBUG_CUT_AXIS)
//      printf("found %d source vertices\n", num_source_verts);
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
        distance_.isSource.values[n] = 1.;
      }
    }  // end setting sources
  }
  // calculate the distances
  printf("\n source indices = ");
  for (int i = 0; i < (int)local_source_indices.size(); i++){
    printf(", %d", local_source_indices[i]);
  }
  printf("\n");

  hmTriDistanceUpdate(&distance_);
//  printf("distances to each row \n");
//  for (int i=0; i<plane_size; i++){
//    printf("\n row %d = ", i);
//    for (int j=20; j<30; j++){
//      printf(", %f", distance_.distance.values[i*plane_size + j]);
//    }
//  }
  hmTriHeatPaths THP(&distance_, config_.raster_spacing);

  THP.compute(&distance_, local_source_indices);

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

  /* deallocate data structures*/
  // TODO there are known memory leaks fix them
  hmTriMeshDestroy(&surface_);
  hmTriDistanceDestroy(&distance_);
  hmContextDestroy(&context_);
}  // end of plan_paths()

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

bool HeatSurfacePlanner::getBoundarySources(const shape_msgs::Mesh& mesh,
                                            std::vector<int>& source_indices,
                                            const size_t expected_size = 3){

}
}  // end of namespace heat
