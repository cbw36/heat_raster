#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/meshio.h"
#include "geometrycentral/surface/surface_mesh.h"

#include "geometrycentral/surface/heat_method_distance.h"
#include <heat_ros/heat_surface_planner.hpp>



using namespace geometrycentral;
using namespace geometrycentral::surface;

int main(int argc, char** argv) {

    // Load a general surface mesh from file
    std::unique_ptr<SurfaceMesh> mesh;
    std::unique_ptr<VertexPositionGeometry> geometry;

    // load the mesh, constructing connectivity & geometry
    std::tie(mesh, geometry) = readSurfaceMesh("/home/cwolfe/grinding_blades_ws/Part Meshes/bunny.obj");

    // alternately, load a mesh which is required to be manifold
    // std::tie(mesh, geometry) = readManifoldSurfaceMesh(args::get(inputFilename));

    // print some information about the mesh
    mesh->printStatistics();

//    // Iterate through the vertices of the mesh, printing the degree of each and the incident faces
//    for (Vertex v : mesh->vertices()) {
//        std::cout << "Vertex " << v << " has degree " << v.degree() << "\n";
//        for (Face fn : v.adjacentFaces()) {
//            std::cout << "  incident on face " << fn << "\n";
//        }
//    }

    Vertex sourceVert = mesh->vertex(0);
    VertexData<double> distToSource = heatMethodDistance(*geometry, sourceVert);

    auto distances = distToSource.toVector();
    std::cout << "size = " << distances.size() << "\n";
    std::cout << "dist 0 = " << distances[0] << "\n";
    std::cout << "dist 1 = " << distances[1] << "\n";
    std::cout << "dist 1000 = " << distances[1000] << "\n";

    return EXIT_SUCCESS;
}
