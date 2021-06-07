#include "geometrycentral/pointcloud/point_cloud.h"
#include "geometrycentral/pointcloud/point_position_geometry.h"
#include "geometrycentral/pointcloud/point_cloud_heat_solver.h"
#include "geometrycentral/pointcloud/point_cloud_io.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/meshio.h"
#include "geometrycentral/surface/vertex_position_geometry.h"
#include "geometrycentral/surface/heat_method_distance.h"
#include "geometrycentral/surface/direction_fields.h"



#include <bits/stdc++.h>

using namespace std;
using namespace geometrycentral;
using namespace geometrycentral::pointcloud;
using namespace geometrycentral::surface;

std::unique_ptr<ManifoldSurfaceMesh> mesh;
std::unique_ptr<VertexPositionGeometry> geometry;

int main(int argc, char **argv) {
//    std::unique_ptr<SurfaceMesh> mesh;
//    std::unique_ptr<VertexPositionGeometry> geometry;
//    std::tie(mesh, geometry) = readSurfaceMesh("/home/cwolfe/heat_method_ws/src/Part Meshes/planar_mesh.ply");

//    mesh->printStatistics();
//
//    pcl::PointCloud<pcl::PointXYZ> cloud;
//
//    // Fill in the cloud data
//    cloud.width    = 257;
//    cloud.height   = 257;
//    cloud.is_dense = false;
//    cloud.points.resize (cloud.width * cloud.height);
//
//    for (int i=0; i<mesh->nVertices(); i++){
//        Vertex v = mesh->vertex(i);
//        Vector3& pos = geometry->inputVertexPositions[v];
//        pcl::PointXYZ point;
//        point.x=pos.x;
//        point.y=pos.y;
//        point.z=pos.z;
//        cloud.push_back(point);
//    }
//
//    pcl::PLYWriter writer;
//    writer.write("/home/cwolfe/heat_method_ws/src/Part Meshes/planar_cloud.ply", cloud);
//    pcl::io::savePCDFileASCII ("/home/cwolfe/heat_method_ws/src/Part Meshes/planar_cloud.pcd", cloud);
//    std::cerr << "Saved " << cloud.size () << " data points to test_pcd.pcd." << std::endl;

//    for (const auto& point: cloud) {
//        std::cerr << "    " << point.x << " " << point.y << " " << point.z << std::endl;
//    }

    // Read in a point cloud
    std::unique_ptr<PointCloud> cloud;
    std::unique_ptr<PointPositionGeometry> geom;
    std::tie(cloud, geom) = readPointCloud("/home/cwolfe/heat_method_ws/src/Part Meshes/planar_cloud.ply");

    std::cout << "cloud has " << cloud->nPoints() << " points.\n";


    // Create the solver
    PointCloudHeatSolver solver(*cloud, *geom);

    // Pick source points
    std::vector<Point> sourceVerts;
    std::vector<double> y_vals;
    std::cout <<"\n";
    for(Point p : cloud->points()) {
        Vector3& point = geom->positions[p];
        y_vals.push_back(point.y);
        if (point.y ==1){
            std::cout << p.getIndex() << ", ";
            sourceVerts.push_back(p);
        }
    }
    sort( y_vals.begin(), y_vals.end() );
    y_vals.erase( unique( y_vals.begin(), y_vals.end() ), y_vals.end() );

    std::vector<std::vector<int>> row_inds;
    for (int i=0;i<y_vals.size(); i++)
    {
        std::vector<int> new_vec;
        row_inds.push_back(new_vec);
    }

    for(Point p : cloud->points()) {
        Vector3& point = geom->positions[p];
        auto it = find(y_vals.begin(), y_vals.end(), point.y);
        if (it != y_vals.end()) {
//            int index = distance(y_vals.begin(), it);
            int index = it - y_vals.begin();
            row_inds.at(index).push_back(p.getIndex());
        }
    }

//     Compute geodesic distance
    PointData<double> distance = solver.computeDistance(sourceVerts);

    for (int i=0;i<row_inds.size();i++) {
        std::cout << "\ndistances to row "<<i<< " points = \n";
        for (int j=row_inds.size()-1; j>row_inds.size()-15; j--) {
            Point p = cloud->point(row_inds.at(i).at(j));
            Vector3& pos = geom->positions[p];
//            printf("%f, ", pos.y);
//            std::cout << distToSource[sourceVerts.at(i).getIndex()] << ", "; //not using this one
            printf("%f, ", distance[row_inds.at(i).at(j)]); //using this one
        }
    }



    //verify elements in pointcloud and mesh are the same
//    for (int i=0; i<mesh->nVertices(); i++){
//        Vertex v = mesh->vertex(i);
//        Vector3& pos = geometry->inputVertexPositions[v];
//        Point point = cloud->point(i);
//        Vector3& point_ = geom->positions[point];
//        if (point_.x!=pos.x){
//            printf("index %d, vertex.x = %f, point.x = %f\n", i, pos.x, point_.x);
//        }
//        if (point_.y!=pos.y){
//            printf("index %d, vertex.y = %f, point.y = %f\n", i, pos.y, point_.y);
//        }
//        if (point_.z!=pos.z){
//            printf("index %d, vertex.z = %f, point.z = %f\n", i, pos.z, point_.z);
//        }
//    }

}
