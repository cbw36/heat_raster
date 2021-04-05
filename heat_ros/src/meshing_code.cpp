#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/visualization/cloud_viewer.h>

#include <pcl/surface/vtk_smoothing/vtk_mesh_quadric_decimation.h>


int main(int argc, char** argv)
{
  // Load input file into a PointCloud<T> with an appropriate type
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PCLPointCloud2 cloud_blob;
  pcl::io::loadPLYFile("/home/cwolfe/Downloads/Meshing/part1_1.ply", cloud_blob);
  pcl::fromPCLPointCloud2(cloud_blob, *cloud);
  std::cout << "size = " << cloud->points.size()
            << ", width = " << cloud->width
            << ", height = " << cloud->height
            << std::endl;
  std::cout << "Finished Loading Mesh" << std::endl;

  // Convert to organized cloud using camera data
  cloud->width = 1026316;
  cloud->height = 1;
  cloud->is_dense = true;

  // Triangulate Mesh
  pcl::OrganizedFastMesh<pcl::PointXYZ > ofm;
  pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh);
  ofm.setMaxEdgeLength(2.5);
  ofm.setTriangulationType(pcl::OrganizedFastMesh<pcl::PointXYZ>::TRIANGLE_ADAPTIVE_CUT);
  ofm.setInputCloud(cloud);
  ofm.reconstruct(*triangles);

  std::cout << "Saving Mesh" << std::endl;
  pcl::io::savePLYFileBinary("/home/cwolfe/Downloads/Meshing/part1_1_meshed.ply", *triangles);

  // This does not seem to work. Possibly there are duplicate vertices and the mesh is not actually "connected"
  std::cout << "Decimating Mesh" << std::endl;
  pcl::MeshQuadricDecimationVTK mqd;
  mqd.setTargetReductionFactor(90.);
  std::cout << "Reduction: " << mqd.getTargetReductionFactor() << std::endl;
  mqd.setInputMesh(triangles);
  pcl::PolygonMesh triangles_small;
  mqd.process(triangles_small);

  std::cout << "Saving Mesh" << std::endl;
  pcl::io::savePLYFileBinary("/home/cwolfe/Downloads/Meshing/part1_1_meshed_small.ply", triangles_small);

  std::cout << "Visualize Mesh" << std::endl;
  // visualize normals
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  viewer.setBackgroundColor(0.0, 0.0, 0.5);
  viewer.addPolygonMesh(triangles_small);

  while (!viewer.wasStopped())
  {
    viewer.spinOnce();
  }

  // Finish
  return (0);
}
