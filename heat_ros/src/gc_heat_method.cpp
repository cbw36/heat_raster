#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/meshio.h"
#include "geometrycentral/surface/surface_mesh.h"
#include "geometrycentral/utilities/vector3.h"

#include "geometrycentral/surface/heat_method_distance.h"
#include <heat_ros/heat_surface_planner.hpp>
#include <geometrycentral/surface/simple_polygon_mesh.h>
#include "geometrycentral/surface/surface_mesh_factories.h"


using namespace geometrycentral;
using namespace geometrycentral::surface;

int main(int argc, char** argv) {

    // Load a general surface mesh from file
//    std::unique_ptr<SurfaceMesh> mesh;
//    std::unique_ptr<VertexPositionGeometry> geometry;

    // load the mesh, constructing connectivity & geometry
//    std::tie(mesh, geometry) = readSurfaceMesh("/home/cwolfe/grinding_blades_ws/src/Part Meshes/plane.ply");
//    mesh->printStatistics();

//    std::vector<Vector3> vertices;

//    Eigen::MatrixXd vMat(1000000,3);
//    Eigen::MatrixXi fMat(2000000, 3);

    int n = 9;
////    SimplePolygonMesh mesh = SimplePolygonMesh();
//    for (int i=0; i<n; i++){
//      for (int j=0; j<n; j++){
//        vMat(i*n + j, 0) = i*0.001;
//        vMat(i*n + j, 1) = j*0.001;
//        vMat(i*n + j, 2) = 0.0;
////        Vector3 v{(double)i, (double)j, 0};
////        mesh.vertexCoordinates.push_back(v);
//      }
//    }
////    std::cout << "rows = "<< vMat.rows() << " and cols = " << vMat.cols() << "\n";
////    printf("done 1");
//    for (int i=0; i<n; i++){
//      for (int j=0; j<n; j++){
//        fMat(i*n + j, 0) = i*n + j;
//        fMat(i*n + j, 1) = i*n + j+1;
//        fMat(i*n + j, 2) = (i+1)*n + j;

//        fMat(i*n + j + 1, 0) = i*n + j+1;
//        fMat(i*n + j + 1, 1) = (i+1)*n + j;
//        fMat(i*n + j + 1, 2) = (i+1)*n + j+1;
////        std::vector<size_t> triangle_1;
////        triangle_1.push_back(i*n + j);
////        triangle_1.push_back(i*n + j+1);
////        triangle_1.push_back((i+1)*n + j);
////        mesh.polygons.push_back(triangle_1);

////        std::vector<size_t> triangle_2;
////        triangle_2.push_back(i*n + j+1);
////        triangle_2.push_back((i+1)*n + j);
////        triangle_2.push_back((i+1)*n + j+1);
////        mesh.polygons.push_back(triangle_2);
//      }
//    }

//    geometrycentral::surface::Vertex v1 = mesh_->vertex(0);
//    geometrycentral::Vector3& pos1 = geometry_->inputVertexPositions[v1];
    printf("1\n");
    std::unique_ptr<SurfaceMesh> mesh;
    printf("2\n");
    std::unique_ptr<VertexPositionGeometry> geometry;
//    std::tie(mesh, geometry) = makeSurfaceMeshAndGeometry(vMat, fMat);
    printf("3\n");
    std::tie(mesh, geometry) = geometrycentral::surface::loadMesh("/home/cwolfe/grinding_blades_ws/src/Part Meshes/grid.ply"); //03.02
    printf("4\n");
    HeatMethodDistanceSolver heatSolver(*geometry);

    mesh->printStatistics();

    std::vector<Vertex> sourceVerts;
    for (int i=0; i<n;i++){
      sourceVerts.push_back(mesh->vertex(i));
    }
    VertexData<double> distToSource = heatSolver.computeDistance(sourceVerts);
    for (int i=0;i<n;i++){
      printf("Distances for row %d\n", i);
      for (int j=0;j<n;j++){
        printf("%f, ", distToSource[i*n+j]);
      }
      printf("\n");
    }

//    writeSurfaceMesh(*mesh, *geometry, "planar_mesh.obj");

//    SurfaceMesh surface_mesh = SurfaceMesh(mesh.polygons);
//    printf("done 2");
//    std::cout << "nvertices = " << mesh.nVertices() << "and nfaces = " << mesh.nFaces() << "\n";
//    std::string filename ("planar_mesh.obj");
//    mesh.writeMesh(filename);
//    printf("done 3");
//    for(int i=0;i<mesh->nVertices();i++){
//      for(int j=0;j<mesh->nVertices();j++){
//       geometrycentral::surface::Vertex v = (i*dx, j*dy, 0.0)
//       vertices.push_back(v);
//     }
//    }

    // more code follows that creates each triangle with for example these set of vertices v[i*n+j], v[i*n + j +1] and  v[(i+1)*n+j], you also need the lower triangle which has v[i*n + j+1], v[(i+1)*n + j] and v[(i+1)*n + j+1]
//    geometrycentral::surface::Vertex v1 = mesh->vertex(0);
//    geometrycentral::surface::Vertex v2 = mesh->vertex(2376);
//    geometrycentral::Vector3& pos1 = geometry->inputVertexPositions[v1];
//    geometrycentral::Vector3& pos2 = geometry->inputVertexPositions[v2];
//    printf("pos 1 = %f, %f, %f, pos 2 = %f, %f, %f", pos1.x, pos1.y, pos1.z, pos2.x, pos2.y, pos2.z);


//    for (size_t i = 0; i < mesh->nFaces(); i++) //TODO unsure if this is right
//    {
//      geometrycentral::surface::Face face = mesh->face(i);
//      printf("face #%d, face degree = %d\n", i, face.degree());
//      int j=1;
//      for(Vertex v : face.adjacentVertices()) {
//        j++;
//        printf("vertex\n");
//      }
//      printf("number adjascent verts = %d\n", j);
//    }

    // alternately, load a mesh which is required to be manifold
    // std::tie(mesh, geometry) = readManifoldSurfaceMesh(args::get(inputFilename));

    // print some information about the mesh
//    mesh->printStatistics();

//    // Iterate through the vertices of the mesh, printing the degree of each and the incident faces
//    for (Vertex v : mesh->vertices()) {
//        std::cout << "Vertex " << v << " has degree " << v.degree() << "\n";
//        for (Face fn : v.adjacentFaces()) {
//            std::cout << "  incident on face " << fn << "\n";
//        }
//    }

//    Vertex sourceVert = mesh->vertex(0);
//    VertexData<double> distToSource = heatMethodDistance(*geometry, sourceVert);

//    auto distances = distToSource.toVector();
//    std::cout << "size = " << distances.size() << "\n";
//    std::cout << "dist 0 = " << distances[0] << "\n";
//    std::cout << "dist 1 = " << distances[1] << "\n";
//    std::cout << "dist 1000 = " << distances[1000] << "\n";

    return EXIT_SUCCESS;
}
