#include "geometrycentral/surface/heat_method_distance.h"
#include "geometrycentral/surface/meshio.h"

// Load a mesh
std::unique_ptr<SurfaceMesh> mesh;
std::unique_ptr<VertexPositionGeometry> geometry;
std::tie(mesh, geometry) = loadMesh("/home/cwolfe/grinding_blades_ws/src/Part Meshes/bunny.obj");

// Pick a vertex
Vertex sourceVert; /* some vertex */

// Compute distance
//VertexData<double> distToSource = heatMethodDistance(*geometry, sourceVert);
/* do something useful */
