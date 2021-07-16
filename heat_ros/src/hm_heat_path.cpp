/**
 * @file
 * @author Chris Lewis <clewis@swri.org>
 * @version 1.0
 *
 * @section LICENSE
 *
 * Copyright 2020 Chris Lewis. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE FREEBSD PROJECT OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation are those
 * of the author and should not be interpreted as representing official policies,
 * either expressed or implied, of any other person or institution.
 *
 */
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <vector>
#include <chrono>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/undirected_dfs.hpp>
#include <boost/graph/breadth_first_search.hpp>
#include <heat_ros/hm_heat_path.hpp>
#include "geometrycentral/utilities/vector3.h"

#include <swri_profiler/profiler.h>



//#define UDGCD_REDUCE_MATRIX
//#include "udgcd.hpp"

#define DEBUG_INFO 1           // print basic information
#define DEBUG_VCHAIN 0         // print each vchain and info
#define DEBUG_VCHAINS 1        // print number of vchains per band
#define DEBUG_INBAND 1         // print each in band info
#define DEBUG_VCHAIN_GRAPHS 1  // print vchain graph info verts and edges
#define DEBUG_SHORT_GRAPH 0    // indicate when a graph is very short
#define DEBUG_DEPTH_FIRST 1    // debug the depth first search for paths
#define DEBUG_REDUCE 0         // debug the depth first search for paths
#define DEBUG_MATLAB 0         // output matlab files for debugging
#define DEBUG_NORMALS 0        // computation of normals
#define DEBUG_QUAT 0           // computaton of quaternion of poses
#define DEBUG_CONCAT 0         // concatenation phase
#define DIST_EXCLUDE 9999999.9

using boost::dijkstra_shortest_paths;

double vertex_distance(std::shared_ptr<geometrycentral::surface::SurfaceMesh> mesh,
                       std::shared_ptr<geometrycentral::surface::VertexPositionGeometry> geometry,
                       int v1_index, int v2_index)
{
  geometrycentral::surface::Vertex v1 = mesh->vertex(v1_index);
  geometrycentral::surface::Vertex v2 = mesh->vertex(v2_index);
  geometrycentral::Vector3& p1 = geometry->inputVertexPositions[v1];
  geometrycentral::Vector3& p2 = geometry->inputVertexPositions[v2];
  return (sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z)));
}

// defined here so it won't be more than once
std::vector<hmTriHeatPaths::VChainGraph::edge_descriptor> hmTriHeatPaths::back_edges_;
std::list<hmTriHeatPaths::vertex_des> hmTriHeatPaths::path_list_;
std::list<hmTriHeatPaths::vertex_des> hmTriHeatPaths::discard_list_;
std::map<int, size_t> hmTriHeatPaths::vmap_;   // A map between vertices in Gs_[i] and a minimal graph GI
std::map<int, size_t> hmTriHeatPaths::ivmap_;  // Inverse map between vertices in Gs_[i] and a minimal graph GI

hmTriHeatPaths::hmTriHeatPaths(std::shared_ptr<geometrycentral::surface::SurfaceMesh> mesh,
                               std::shared_ptr<geometrycentral::surface::VertexPositionGeometry> geometry,
                               std::vector<int> source_indices,
                               geometrycentral::surface::VertexData<double> dist_to_source,
                               double raster_spacing, double time) :
  mesh_(mesh), geometry_(geometry), source_indices_(source_indices), dist_to_source_(dist_to_source), time_(time)
{
  // find maximum distance in mesh, assume 0 is min distance, this may be incorrect, if mean is removed.
  Eigen::Matrix<double, -1, 1> dist_to_source_vec = dist_to_source.toVector();
  max_distance_ = std::numeric_limits<double>::min();
  int num_neg = 0;
  std::cout << "dist_to_source_vec size = " << dist_to_source_vec.size() << "\n";
  for (int i=0; i<dist_to_source_vec.size(); i++){
    if (dist_to_source_vec[i] < 0.0)
    {
      num_neg++;
    }
    if (dist_to_source_vec[i] > max_distance_)
      max_distance_ = dist_to_source_vec[i];
  }
  if (num_neg > 0)
  {
    printf("WARNING num_neg = %d\n", num_neg);
  }
  num_levels_ = (int)(max_distance_ / raster_spacing) + 1;

////  inband_vertex_lists_ = (std::vector<int>)malloc(num_levels_ * sizeof(hmVectorSizeT));

//  double time; //TODO How to compute the average edge length? Vertices dont have coordinates
  double alt_eps = sqrt(time) / 1.7;  // TODO time is the average edge length squared, we want a portion of this
//  double alt_eps = 4.3; //TODO replace. look in hmTriDistance.c void hmTriDistanceEstimateTime(hmTriDistance* distance)
//  double alt_eps = 0.016300; //TODO replace
  epsilon_ = raster_spacing / 28;
  //  epsilon_ = raster_spacing / 7.5;

  if (alt_eps > epsilon_)
    epsilon_ = alt_eps;
  delta_ = raster_spacing;

// create matrix of faces incident on each vertex. TODO may not need this bc gc implements this
//  for (int i=0; i<mesh_->nVertices(); i++){
//    std::vector<int> cur_vec;
//    vert_faces_.push_back(cur_vec);
//  }

//  for(geometrycentral::surface::Face f : mesh_->faces()) {
//  }

  if (DEBUG_INFO)
  {
    printf("max_dist = %f\n", max_distance_);
    printf("num_levels = %d\n", num_levels_);
    printf("delta  = %f\n", delta_);
    printf("epsilon  = %f\n", epsilon_);
    printf("alt_eps = %f\n", alt_eps);
    printf("tine = %f\n", time);
  }
}

char hmTriHeatPaths::is_inband(size_t vertex_index, double band)
{
  SWRI_PROFILE("is_inband");
  double band_min = band - epsilon_;
  double band_max = band + epsilon_;

  // return all source points if band = 0.0
  if ((std::count(source_indices_.begin(), source_indices_.end(), vertex_index)) && (band == 0.0)) //TODO this would return non-source elements with distance=0.0. Is that okay?
                                                              //Could alternatively find if vertex_index is in sources
  {
    return (1);
  }
  //TODO returning 0 below for all nonsource points within the source band leads to 36 vchains for band 0 rather than 1
  else if (band == 0.0) {
    return (0);
  }
  // check threshold on others
  if (dist_to_source_[vertex_index] >= band_min && dist_to_source_[vertex_index] < band_max)
  {
    return (1);
  }
  return (0);
}

void hmTriHeatPaths::remove_matching_vertices(std::vector<int> &to_match, std::vector<int> &vertex_list)
{
  std::vector<int> no_match_vertices;
  for (int i = 0; i < (int)vertex_list.size(); i++)
  {
    int match = 0;
    for (int j = 0; j < (int)to_match.size(); j++)
    {
      if (to_match.at(j) == vertex_list.at(i))
      {
        match = 1;
        continue;
      }
    }  // end for all matches
    if (match == 0)
    {
      no_match_vertices.push_back(vertex_list.at(i));
    }
  }  // end for all vertices in list
  move_vertices(no_match_vertices, vertex_list);
}

void hmTriHeatPaths::move_vertices(std::vector<int>& from, std::vector<int>& to){
  to = from;
  from.clear();
}


bool hmTriHeatPaths::inVector(std::vector<int> vec, int el){
  if (std::find(vec.begin(), vec.end(), el) != vec.end())
    return true;
  else
    return false;
}

char hmTriHeatPaths::extract_vchain(std::vector<int> &vertex_list,
                                    double band,
                                    std::vector<int> &vchain)
{
  SWRI_PROFILE("extract_vchain");
  if (vertex_list.size() == 0)
    return (0);

  std::vector<int> old_inband_neighbors;
  std::vector<int> new_inband_neighbors;
  old_inband_neighbors.push_back(vertex_list.at(0)); //start vchain with first vertex in the current band

  while (old_inband_neighbors.size() > 0){
    for (int i=0; i<(int)old_inband_neighbors.size(); i++){
      geometrycentral::surface::Vertex current_vertex = mesh_->vertex(old_inband_neighbors.at(i));
//      printf("old inband_neighbors size = %d, i=%d, num_neighbors = %d\n", old_inband_neighbors.size(), i, current_vertex.degree());
      for (geometrycentral::surface::Vertex neighbor : current_vertex.adjacentVertices()){
        int neighbor_ind = neighbor.getIndex();
        if (is_inband(neighbor_ind, band) && !inVector(old_inband_neighbors, neighbor_ind) &&
            !inVector(vchain, neighbor_ind) && !inVector(new_inband_neighbors, neighbor_ind)){
          new_inband_neighbors.push_back(neighbor_ind);
        }
      }
      vchain.push_back(old_inband_neighbors.at(i));
    }
    move_vertices(new_inband_neighbors, old_inband_neighbors);
  }
//  printf("before vertex list size = %d\n", vertex_list.size());
  remove_matching_vertices(vchain, vertex_list);
//  printf("after vertex list size = %d\n", vertex_list.size());
  return (1);
}

void hmTriHeatPaths::face_normal(size_t vertex_index, Eigen::Vector3d& normal_vec)
{
  SWRI_PROFILE("face_normal");
  auto start = std::chrono::high_resolution_clock::now();
  num_face_norm_calls_ ++;
  // find all faces containing this vertex
  geometrycentral::surface::Vertex vert = mesh_->vertex(vertex_index);

//  std::vector<size_t> included_faces;

//  for (size_t i = 0; i < mesh_->nFaces(); i++) //TODO unsure if this is right
//  {
//    geometrycentral::surface::Face face = mesh_->face(i);
//    bool vert_on_face = false;
//    for (geometrycentral::surface::Vertex v : face.adjacentVertices()){
//      if (v.getIndex() == vertex_index){
//        vert_on_face = true;
//      }
//    }
//    if (vert_on_face)
//    {
//      included_faces.push_back(i);
//    }
//  }

  // find average normal of all faces
  normal_vec(0) = 0.0;
  normal_vec(1) = 0.0;
  normal_vec(2) = 0.0;
//  double* vertices = distance->surface->vertices;
//  for (int i = 0; i < (int)included_faces.size(); i++)
  for (geometrycentral::surface::Face face : vert.adjacentFaces())
  {
//    size_t* face = &faces[included_faces[i] * 3];
//    geometrycentral::surface::Face face = mesh_->face(i);

    //TODO unsure if this approach is same as what Chris calculated and if the order of vertices is the same.
    //can instead use DenseMatrix<size_t> F which returns an FxD matrix
    //of vertex indices for each face in mesh, assuming each face has degree D
    std::vector<geometrycentral::Vector3> adj_verts;
    for(geometrycentral::surface::Vertex v : face.adjacentVertices()) {
      geometrycentral::Vector3 vec = geometry_->inputVertexPositions[v];
      adj_verts.push_back(vec);
    }
    Eigen::Vector3d pt1(adj_verts.at(0).x, adj_verts.at(0).y, adj_verts.at(0).z);
    Eigen::Vector3d pt2(adj_verts.at(1).x, adj_verts.at(1).y, adj_verts.at(1).z);
    Eigen::Vector3d pt3(adj_verts.at(2).x, adj_verts.at(2).y, adj_verts.at(2).z);
    Eigen::Vector3d v1 = pt2 - pt1;
    Eigen::Vector3d v2 = pt3 - pt2;
    Eigen::Vector3d nv = v1.cross(v2);
    nv.normalize();
    if (DEBUG_NORMALS)
    {
      printf("pt1 = [%6.3lf %6.3lf %6.3lf];\n", pt1(0), pt1(1), pt1(2));
      printf("pt2 = [%6.3lf %6.3lf %6.3lf];\n", pt2(0), pt2(1), pt2(2));
      printf("pt3 = [%6.3lf %6.3lf %6.3lf];\n", pt3(0), pt3(1), pt3(2));
      printf("nv = [%6.3lf %6.3lf %6.3lf];\n", nv(0), nv(1), nv(2));
    }
    normal_vec = normal_vec + nv;
  }
  normal_vec.normalize();  // re-normalize should be same as divide by number of normals
  auto stop = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  std::cout << "Time taken by function: " << duration.count() << "\n";
}

void hmTriHeatPaths::build_vchain_graph(std::vector<int> &vchain, hmTriHeatPaths::VChainGraph& G)
{
  SWRI_PROFILE("build_vchain_graph");
  int no_chain_neighbors_count = 0;
  IndexMap index_map = get(boost::vertex_index, G);
  boost::property_map<VChainGraph, boost::edge_weight_t>::type weight = boost::get(boost::edge_weight, G);

//  const hmVectorPairSizeTDouble* VN = distance->surface->vertexNeighbors;
//  const hmPairSizeTDouble *neighborsBegin, *neighborsEnd, *currentNeighbor;
//  geometrycentral::surface::VertexSet vertices = mesh_->vertices();
  int n_edges = 0;
  if (DEBUG_VCHAIN)
    printf("vchain graph [%ld", vchain.at(0));

  for (int j = 0; j < (int)vchain.size(); j++)  // for each vertex in the chain
  {
    size_t v_idx = (size_t)vchain.at(j);
    geometrycentral::surface::Vertex current_vertex = mesh_->vertex(v_idx);

    for (geometrycentral::surface::Vertex neighbor : current_vertex.adjacentVertices()) // for each neighbor of the vertex
    {
      int ni = neighbor.getIndex();
      bool neighbor_in_chain = false;
      for (int k = 0; k < (int)vchain.size(); k++)
      {
        if (vchain.at(k) == ni)
          neighbor_in_chain = true;
      }
      if (neighbor_in_chain)
      {
        double length = vertex_distance(mesh_, geometry_, v_idx, ni);
        size_t from = v_idx, to = ni;
        if (v_idx > ni)  // use smaller as from
        {
          from = ni;
          to = v_idx;
        }

        VChainGraph::edge_descriptor e1;
        bool edge_found = false, rtn;
        if (n_edges != 0)  // search for edge fails when G is empty
          boost::tie(e1, edge_found) = boost::edge(from, to, G);
        if (!edge_found)
        {
          boost::tie(e1, rtn) = boost::add_edge(from, to, length, G);
          n_edges = (int)num_edges(G);
          if (rtn == true && DEBUG_VCHAIN)
          {
            if (DEBUG_VCHAIN)
            {
              printf(",%ld-%ld", index_map[boost::source(e1, G)], index_map[boost::target(e1, G)]);
              printf("(%5.2lf) ", weight(e1));
            }
          }
        }  // end if edge is not found
      }    // end if neighbor_in_chain
      else
      {
        no_chain_neighbors_count++;
      }
    }
  }
  if (DEBUG_VCHAIN)
    printf(
        "] \n vchain->size = %ld but %d vertices without neighbors in chain\n", vchain.size(), no_chain_neighbors_count);
}

void hmTriHeatPaths::compute_inband_verticies(const std::vector<int>& sources)
{
  SWRI_PROFILE("compute_inband_vertices");
  // add sources as inband_vertex_lists_[num_levels-1]
  std::vector<int> band_1;
  for (int i = 0; i < (int)sources.size(); i++){
    band_1.push_back(sources[i]);
  }
  inband_vertex_lists_.push_back(band_1);
  if (DEBUG_INBAND)
    printf("%d band %f has %ld vertices: \n", 0, 0.00000, band_1.size());

  // add all other bands
  for (int i = 1; i < (int)num_levels_; i++)
  {
    double band = i * delta_;
//    // initialize the vertex list for each distance level
    std::vector<int> new_band;
//    // compute the band
    for (size_t j = 0; j < mesh_->nVertices(); j++)
    {
      if (is_inband(j, band))
      {
        new_band.push_back(j);
      }  // end if vertex within band
    }    // end for each vertex
    inband_vertex_lists_.push_back(new_band);
    if (DEBUG_INBAND)
    {
      printf("%d band %f has %ld vertices: [", i, band, new_band.size());
//      for (int q = 0; q < new_band.size(); q++)
//      {
//        printf("%ld ", new_band[q]);
//      }
    }
    if (DEBUG_INBAND)
      printf("]\n");
  }  // end for every level
}

void hmTriHeatPaths::compute_vchains()
{
  SWRI_PROFILE("compute_vchains");
  for (int i = 0; i < (int)num_levels_; i++)
  {
    int old_vchain_size = vcs_.size();
    double band = i * delta_;
    while (inband_vertex_lists_.at(i).size() != 0)
    {
      std::vector<int> cur_vchain;
      if (extract_vchain(inband_vertex_lists_.at(i), band, cur_vchain))
      {
        if (DEBUG_VCHAINS)
        {
//          printf("vchain = [ ");
//          for (int q = 0; q < (int)cur_vchain.size(); q++)
//            printf("%ld ", cur_vchain.at(q));
//          printf("]\n");
        }
          vcs_.push_back(cur_vchain);
      }
      else
      {
        printf("could not extract vchain\n");
      }
    }
    if (DEBUG_VCHAINS){
      printf("band = %lf to %lf has %ld vchain, with ", band - epsilon_, band + epsilon_, vcs_.size() - old_vchain_size);
      for (int i=old_vchain_size; i<vcs_.size(); i++)
        printf("%d, ", vcs_.at(i).size());
      printf(" vertices in each chain \n");
    }
  }
}

void hmTriHeatPaths::compute_vchain_graphs()
{
  SWRI_PROFILE("compute_vchain_graphs");
  for (int i = 0; i < (int)vcs_.size(); i++)
  {
    VChainGraph G;
    build_vchain_graph(vcs_.at(i), G);
    if (DEBUG_VCHAIN_GRAPHS)
      printf("vchain graph %d has %ld edges\n", i, boost::num_edges(G));  // boost::num_vertices(G), #vertices is a bad
                                                                          // indication of graph size
    Gs_.push_back(G);
  }
}

void hmTriHeatPaths::reduce_graph_exclude_near_verts(VChainGraph& G_in,
                                                     VChainGraph& G_out,
                                                     const std::vector<vertex_des>& V_e)
{
  SWRI_PROFILE("reduce_graph_exclude_near_verts");
  boost::property_map<VChainGraph, boost::edge_weight_t>::type weightmap = boost::get(boost::edge_weight, G_in);
  VChainGraph::edge_iterator ei, ee;
  IndexMap index_map = get(boost::vertex_index, G_in);
  std::list<size_t> vlist;
  std::vector<size_t> sources;
  std::vector<size_t> targets;
  std::vector<double> weights;
  for (boost::tie(ei, ee) = boost::edges(G_in); ei != ee; ei++)
  {
    size_t v1 = index_map[boost::source(*ei, G_in)];
    size_t v2 = index_map[boost::target(*ei, G_in)];
    bool add_this_edge = true;
    for (size_t i = 0; i < V_e.size(); i++)
    {  // check to see if edge is close to previous path
      double d1 = vertex_distance(mesh_, geometry_, (int)V_e[i], v1);
      double d2 = vertex_distance(mesh_, geometry_, (int)V_e[i], v2);
      if (d1 < epsilon_ || d2 < epsilon_)
      {
        add_this_edge = false;
        break;
      }
    }
    if (add_this_edge)
    {
      double edge_length = weightmap(*ei);
      vlist.push_back(v1);
      vlist.push_back(v2);
      sources.push_back(v1);
      targets.push_back(v2);
      weights.push_back(edge_length);
    }
  }                // end for each edge in G_in
  vlist.sort();    // sort list, so that unique() can delete duplicates
  vlist.unique();  // now we have a minimal list of vertices in Gs_[i] that have edges

  // now we create a mapping between the vertices in Gs_[i] and the new vertices in G
  vmap_.clear();   // create a map between vertices in G_in and a new graph G_out
  ivmap_.clear();  // create a inverse map between G_out and G_in
  int q = 0;
  for (size_t n : vlist)
  {
    vmap_.emplace(q, n);  // index q, value n, vertex g(q) is equivailent to vertex G(n)
    int in = (int)n;
    size_t qs = (size_t)q;
    ivmap_.emplace(in, qs);  // index n, value q
    q++;
  }
  // Create G_out using only those vertices in
  for (size_t i = 0; i < sources.size(); i++)
  {
    boost::add_edge(ivmap_[sources[i]], ivmap_[targets[i]], weights[i], G_out);
  }
  if (DEBUG_REDUCE)
    printf("reduced graph now has %ld verts\n", num_vertices(G_out));

  // we now have G_out to return, and ivmap_ and vmap_ setup
}

std::vector<hmTriHeatPaths::vertex_des> hmTriHeatPaths::find_path(VChainGraph& G)
{
  SWRI_PROFILE("find_paths");
  std::vector<vertex_des> p_temp;  // path_list as a vector

  // do nothing if its a small graph
  if (boost::num_vertices(G) < 3 || boost::num_edges(G) < 2)
  {
    if (DEBUG_SHORT_GRAPH)
      printf("No path can be found with num_vertices < 3\n");
    return (p_temp);
  }

  // vertex color map for boost::undirected_dfs
  std::vector<boost::default_color_type> vertex_color(boost::num_vertices(G));
  auto index_map = boost::get(boost::vertex_index, G);
  auto vcmap = make_iterator_property_map(vertex_color.begin(), index_map);

  // instantiate a path generator for boost::breadth_first_search()
  PathGen pathGen(mesh_, geometry_, time_);
  pathGen.Clear();  // old path list
  VChainGraph::vertex_iterator s, e;
  boost::tie(s, e) = boost::vertices(G);
  boost::queue<double> Q;
  boost::breadth_first_search(G, *s, Q, pathGen, vcmap);  // do a breadth_first_search

  // check if bfs is successful
  if (!pathGen.path_found())
  {
    printf("find_path() breadth_first_search() failed. Graph had %ld vertices\n", num_vertices(G));
    return (p_temp);
  }

  // connect tail to head if they are close
  double d = vertex_distance(mesh_, geometry_, path_list_.front(), path_list_.back());
  if (d < sqrt(epsilon_))
    path_list_.push_back(path_list_.front());

  for (vertex_des n : path_list_)
  {
    p_temp.push_back(n);
  }
  vertex_sequences_.push_back(p_temp);

  return (p_temp);
}


std::list<hmTriHeatPaths::vertex_des>::iterator hmTriHeatPaths::PathGen::find_closest_point(std::list<vertex_des>& list,
                                                                                            int v_index,
                                                                                            double& cd)
{
  SWRI_PROFILE("find_closest_point");
  std::list<vertex_des>::iterator closest_it;
  double d = vertex_distance(mesh_, geometry_, v_index, path_list_.front());
  cd = d;
  closest_it = list.begin();
  for (std::list<vertex_des>::iterator it = list.begin(); it != list.end(); it++)
  {
    d = vertex_distance(mesh_, geometry_, v_index, *it);
    if (cd < d)
    {
      cd = d;
      closest_it = it;
    }
  }  // end for every vertex already on list
  return (closest_it);
}

void hmTriHeatPaths::PathGen::discover_vertex(vertex_des& v, const VChainGraph& G)
{
  // convert vertex index from reduced graph (gi) into original mesh vertex index (vi)
  // find distance between vertex vi and both the top and bottom of the list
  // add vertex to top or bottom depending on which is closer

  int vi = vmap_[v];  // mesh index of vertex
  if (path_list_.size() < 1)
  {
    path_list_.push_front(vi);
    return;
  }

  double dt = vertex_distance(mesh_, geometry_, vi, path_list_.front());  // distance to top
  double db = vertex_distance(mesh_, geometry_, vi, path_list_.back());   // distance to bottom
  VChainGraph::edge_descriptor e;
  double thresh = ave_edge_length * 2.0;  // epsilon is usually some portion of the average edge length
  double cd;                              // closest point's distance
  std::list<vertex_des>::iterator closest_it = find_closest_point(path_list_, vi, cd);
  if (dt > db)  // closer to bottom
  {
    if (db < thresh)  // close enough to bottom
    {
      path_list_.push_back(vi);
    }
    else  // not very close to either top or bottom
    {
      discard_list_.push_back(vi);
    }
  }     // end closer to bottom
  else  // closer to top
  {
    if (dt < thresh)
    {
      path_list_.push_front(vi);
    }
    else  // not very close to either top or bottom
    {
      discard_list_.push_front(vi);
    }
  }  // end closer to front
}  // end PathGen::discover_vertex()


void hmTriHeatPaths::compute_depth_first_paths()
{
  SWRI_PROFILE("compute_depth_first_paths");
  printf("ENTER COMPUTE DEPTH FIRST PATHS 00000000\n");
  int tot_num_in_vert_seq = 0;
  for (int i=0; i<vertex_sequences_.size(); i++){
    tot_num_in_vert_seq += vertex_sequences_.at(i).size();
  }
  printf("Number of vertex sequences = %d with total of %d vertices\n", vertex_sequences_.size(), tot_num_in_vert_seq);
  // for every vchain graph, create an equivalent graph Gs_[i] with the same edges, but re-indexed vertices to minimize
  // number of verts in G_reduced exclude any vertex in Gs_[i] that is near any other paths already found
  for (size_t i = 0; i < Gs_.size(); i++)
  {
    std::vector<vertex_des> v_exclude;
    bool done = false;
    while (!done)  // while there are still edges in G_reduced find another path
    {
      VChainGraph G_reduced;
      reduce_graph_exclude_near_verts(Gs_[i], G_reduced, v_exclude);
      std::vector<vertex_des> new_path = find_path(G_reduced);
      if (DEBUG_REDUCE)
        printf("new_path from %ldth vchain graph has %ld points\n", i, new_path.size());
      if (new_path.size() == 0)
        done = true;
      for (size_t i = 0; i < new_path.size(); i++)
      {
        v_exclude.push_back(new_path[i]);
      }
    }
  }  // end for every vchain graph

  if (DEBUG_DEPTH_FIRST)
  {
    int tot_num_in_vert_seq = 0;
    for (int i=0; i<vertex_sequences_.size(); i++){
      tot_num_in_vert_seq += vertex_sequences_.at(i).size();
    }
    printf("Number of vertex sequences = %d with total of %d vertices\n", vertex_sequences_.size(), tot_num_in_vert_seq);
//    for (size_t i = 0; i < vertex_sequences_.size(); i++)
//    {
//      printf("vertex_sequences_[%ld] %ld vertices\n", i, vertex_sequences_[i].size());
//    }
  }
}
void hmTriHeatPaths::concatenate_paths(int path_i, int path_j, bool invert_j)
{
  SWRI_PROFILE("concatenate_paths");
  int s = (int)vertex_sequences_[path_j].size();
  if (!invert_j)
  {
    for (int j = 0; j < s; j++)
    {
      vertex_sequences_[path_i].push_back(vertex_sequences_[path_j][j]);
    }
  }
  else
  {  // invert path j
    for (int j = s; j > 0; j--)
    {
      vertex_sequences_[path_i].push_back(vertex_sequences_[path_j][j - 1]);
    }
  }
}

void hmTriHeatPaths::compute_pose_arrays()
{
  SWRI_PROFILE("compute_pose_arrays");
  // create a set of poses from the sequence by:
  // 1. Use surface normal for z_vec of pose
  // 2. Use direction to next in sequence as x_vec
  // 3. Use z_vec cross x_vec as y
  // 4. compute quaternion from 3x3 rotation defined by x_vec|y_vec|z_vec
  // 5. use most recent quat for last in sequence
  printf("Enter compute_pose_arrays\n");
  pose_arrays_.clear();
  int NV = mesh_->nVertices();
  printf("NV = %d\n", NV);
  int tot_num_in_vert_seq = 0;
  for (int i=0; i<vertex_sequences_.size(); i++){
    tot_num_in_vert_seq += vertex_sequences_.at(i).size();
  }
  printf("Number of vertex sequences = %d with total of %d vertices\n", vertex_sequences_.size(), tot_num_in_vert_seq);
  std::vector<std::vector<int>> index_occurences(vertex_sequences_.size());
  for (int i = 0; i < (int)vertex_sequences_.size(); i++)
  {
    std::vector<int> cur_occurences(NV, 0);
    std::vector<Pose> Pose_array;
    Pose P;
    printf("sequence # %d size = %d\n", i, vertex_sequences_[i].size());
    for (int j = 0; j < (int)vertex_sequences_[i].size() - 1; j++)
    {
      Eigen::Vector3d z_vec;
      double zv[3];
      int vj = vertex_sequences_[i][j];
      cur_occurences.at(vj) ++;
      int vn = vertex_sequences_[i][j + 1];
      face_normal(vj, z_vec);  // most reliable
      if (DEBUG_QUAT)
        printf("zv = [ %6.3lf %6.3lf %6.3lf]\n", z_vec(0), z_vec(1), z_vec(2));
      geometrycentral::surface::Vertex v1 = mesh_->vertex(vj); //TODO Check I am doing this right. Unsure if my v1 aligns with his
      geometrycentral::surface::Vertex v2 = mesh_->vertex(vn);
      geometrycentral::Vector3& pos1 = geometry_->inputVertexPositions[v1];
      geometrycentral::Vector3& pos2 = geometry_->inputVertexPositions[v2];

      Eigen::Vector3d p1(pos1.x, pos1.y, pos1.z);
      Eigen::Vector3d p2(pos2.x, pos2.y, pos2.z);
      Eigen::Vector3d x_vec = p2 - p1;
      x_vec = x_vec - x_vec.dot(z_vec) * z_vec;
      if (x_vec.norm() != 0.0)
      {
        x_vec.normalize();
        Eigen::Vector3d y_vec = z_vec.cross(x_vec);
        y_vec.normalize();

        // the point is the vertex
        P.x = p1[0];
        P.y = p1[1];
        P.z = p1[2];
        Eigen::Matrix3d R(3, 3);
        R(0, 0) = x_vec(0);
        R(0, 1) = y_vec(0);
        R(0, 2) = z_vec(0);
        R(1, 0) = x_vec(1);
        R(1, 1) = y_vec(1);
        R(1, 2) = z_vec(1);
        R(2, 0) = x_vec(2);
        R(2, 1) = y_vec(2);
        R(2, 2) = z_vec(2);
        Eigen::Quaterniond Q(R);
        if (DEBUG_QUAT)
        {
          printf("Q:%6.3lf %6.3lf %6.3lf %6.3lf\n", Q.w(), Q.x(), Q.y(), Q.z());
          printf("R = [%6.3lf %6.3lf %6.3lf; \n     %6.3lf %6.3lf %6.3lf; \n     %6.3lf %6.3lf %6.3lf]\n",
                 R(0, 0),
                 R(0, 1),
                 R(0, 2),
                 R(1, 0),
                 R(1, 1),
                 R(1, 2),
                 R(2, 0),
                 R(2, 1),
                 R(2, 2));
        }
        P.qw = Q.w();
        P.qx = Q.x();
        P.qy = Q.y();
        P.qz = Q.z();
        Pose_array.push_back(P);
      }
      else
      {
        printf("normalize of x failed for vertex %d in sequence %d\n", j, i);
      }
    }
    index_occurences.at(i) = cur_occurences;

    if (vertex_sequences_[i].size() > 1)
    {
      // add last point in sequence repeating latest orientation
      int vj = vertex_sequences_[i][vertex_sequences_[i].size() - 1];
      if (vj < NV && vj >= 0)
      {
        geometrycentral::surface::Vertex v = mesh_->vertex(vj); //TODO Check I am doing this right. Unsure if my v1 aligns with his
        geometrycentral::Vector3& pos = geometry_->inputVertexPositions[v];
        P.x = pos.x;
        P.y = pos.y;
        P.z = pos.z;
        Pose_array.push_back(P);
        pose_arrays_.push_back(Pose_array);
      }
    }
  }  // end for each vertex sequence
//  std::vector<int> mult_seq_occurences(NV, 0);
//  for (int i=0; i<vertex_sequences_.size(); i++){
//    for (int j=0; j<NV; j++){
//      if (index_occurences.at(i).at(j) != 0){
//        mult_seq_occurences.at(j) ++;
//        printf("sequence %d vertex %d = %d\n", i, j, index_occurences.at(i).at(j));
//      }
//    }
//  }
//  for (int i=0; i<vertex_sequences_.size(); i++){
//    if (mult_seq_occurences.at(i) > 1){
//      printf("vertex %d is in %d sequences\n", i, mult_seq_occurences.at(i));
//    }
//  }
}

void hmTriHeatPaths::connect_paths()
{
  SWRI_PROFILE("connect_paths");
  printf("inside connect_paths!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
  std::vector<bool> marked_deleted;
  int tot_num_in_vert_seq = 0;
  for (int i=0; i<vertex_sequences_.size(); i++){
    tot_num_in_vert_seq += vertex_sequences_.at(i).size();
  }
  printf("vertex_sequences.size() = %d with a total of %d verts\n", (int)vertex_sequences_.size(), tot_num_in_vert_seq);

  //TODO All marked_deleted are false!
  for (int i = 0; i < (int)vertex_sequences_.size(); i++)
  {
    if (vertex_sequences_[i].size() == 0)
    {
      marked_deleted.push_back(true);
    }
    else
    {
      marked_deleted.push_back(false);
    }
  }
  for (int i = 0; i < (int)vertex_sequences_.size(); i++)
  {
    if (!marked_deleted[i])
    {
      for (int j = i + 1; j < (int)vertex_sequences_.size(); j++)
      {
        if (!marked_deleted[j])
        {
          int j_front = (int)vertex_sequences_[j][0];
          int j_end = (int)vertex_sequences_[j][(int)vertex_sequences_[j].size() - 1];
          int i_front = (int)vertex_sequences_[i][0];
          int i_end = (int)vertex_sequences_[i][(int)vertex_sequences_[i].size() - 1];
          ;
          double con_thresh = delta_ / 1.3;
          if (vertex_distance(mesh_, geometry_, i_end, j_front) < con_thresh)  // end of i to front of j
          {
            if (DEBUG_CONCAT)
              printf("concat %d %d\n", i, j);
            concatenate_paths(i, j, false);
            marked_deleted[j] = true;
          }
          else if (vertex_distance(mesh_, geometry_, i_end, j_end) < con_thresh)  // end of i to end of j
          {
            if (DEBUG_CONCAT)
              printf("concat %d %d true\n", i, j);
            concatenate_paths(i, j, true);
            marked_deleted[j] = true;
          }
          else if (vertex_distance(mesh_, geometry_, j_end, i_end) < con_thresh)  // end of j to end of i
          {
            if (DEBUG_CONCAT)
              printf("concat %d %d true\n", j, i);
            concatenate_paths(j, i, true);
            marked_deleted[i] = true;
          }
          else if (vertex_distance(mesh_, geometry_, j_end, i_front) < con_thresh)  // end of j to front of i
          {
            if (DEBUG_CONCAT)
              printf("concat %d %d\n", j, i);
            concatenate_paths(j, i, false);
            marked_deleted[i] = true;
          }
        }  // end if j !marked_deleted()
      }    // end of j loop on vertex_sequences_
    }      // end if i !marked_deleted()
  }        // end of i loop on vertex_sequences_
  for (int i = (int)vertex_sequences_.size(); i > 0; i--)
  {
    if (marked_deleted[i - 1])
    {
      vertex_sequences_.erase(vertex_sequences_.begin() + i - 1);
    }
  }
}

void hmTriHeatPaths::compute(const std::vector<int>& source_indices)
{
  printf("enter compute\n");
  compute_inband_verticies(source_indices);
  printf("finish compute_inband_vertices\n");
  compute_vchains(); //makes vcs_ for the chain x has y vertices line. Get larger results here
  printf("finish compute_vchains\n");
  if (DEBUG_VCHAINS)
  {
    int tot_vert_in_vchains = 0;
    for (int i = 0; i < vcs_.size(); i++)
    {
      tot_vert_in_vchains += vcs_.at(i).size();
//      printf("chain %d has %ld vertices\n", i, vcs_.at(i).size());
    }
    printf("total number of vertices in all vchains is %ld \n", tot_vert_in_vchains);
  }
  compute_vchain_graphs();
  printf("finish compute_vchain_graphs\n");
  compute_depth_first_paths(); //TODO This takes way too long. makes vertex_sequences have 271 items
  printf("finish compute_depth_first_paths\n");
  connect_paths(); //prunes vertex sequences to 39 items
  printf("finish connect_paths\n");
//  /* TODO REMOVE THIS AND THE SUBROUTINE ONCE ALL ELSE WORKS
//    add_sources_to_sequences(distance, source_indices);// don't rely on the 0-band, it does not include these vertices
//    and will be noisy
//  */
  compute_pose_arrays(); //TODO this takes way too long
  printf("num face normal calls = %d\n", num_face_norm_calls_);
  printf("FINISHED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!11");
//  if (DEBUG_MATLAB)
//    octave_output_paths("paths.m", distance);
}  // end of compute







/*
//void hmTriHeatPaths::add_sources_to_sequences(std::shared_ptr<geometrycentral::surface::SurfaceMesh> mesh, const std::vector<int>& source_indices)
//{
////  std::vector<vertex_des> p_temp;  // covert path_list to a vector
////  for (int i = 0; i < (int)source_indices.size(); i++)
////  {
////    p_temp.push_back(source_indices[i]);
////  }
////  vertex_sequences_.push_back(p_temp);
//}

//bool hmTriHeatPaths::compute_dijkstras_on_graph(vertex_des& s,
//                                                VChainGraph& G,
//                                                std::vector<double>& d,
//                                                std::vector<vertex_des>& p)
//{
////  size_t num_verts = num_vertices(G);
////  size_t num_edges = boost::num_edges(G);
////  p.resize(num_verts);
////  d.resize(num_verts);
////  if (num_edges > 1)
////  {
////    boost::property_map<VChainGraph, boost::edge_weight_t>::type weightmap = boost::get(boost::edge_weight, G);
////    boost::property_map<VChainGraph, boost::vertex_index_t>::type index_map = get(boost::vertex_index, G);
////    dijkstra_shortest_paths(G,      // const Graph& g
////                            s,      // vertex_descriptor s, source, all shortest path distances computed from here
////                            &p[0],  // predecessorMap each vertex p[i] points the next closer one p[p[i]] the next after
////                            &d[0],  // distanceMap result providing the distance to each vertex from s
////                            weightmap,  // weightMap accesses the weight of each edge
////                            index_map,  // takes a vertex descriptor and finds the index of the vertex
////                            std::less<double>(),
////                            boost::closed_plus<double>(),
////                            DIST_EXCLUDE,
////                            //			      (std::numeric_limits<double>::max)(),
////                            0.0,
////                            boost::default_dijkstra_visitor());
////  }
////  else
////    return false;
////  return true;
//}


//void hmTriHeatPaths::one_face_normal(std::shared_ptr<geometrycentral::surface::SurfaceMesh> mesh, size_t vertex_index, Eigen::Vector3d& normal_vec)
//{
////  // find all faces containing this vertex
////  std::vector<size_t> included_faces;
////  size_t* faces = distance->surface->faces;
////  for (size_t i = 0; i < distance->surface->nFaces; i++)
////  {
////    size_t* face = &faces[i * 3];
////    if (face[0] == vertex_index || face[1] == vertex_index || face[2] == vertex_index)
////    {
////      included_faces.push_back(i);
////      break;
////    }
////  }

////  // find average normal of all faces
////  normal_vec(0) = 0.0;
////  normal_vec(1) = 0.0;
////  normal_vec(2) = 0.0;
////  double* vertices = distance->surface->vertices;
////  for (int i = 0; i < (int)included_faces.size(); i++)
////  {
////    size_t* face = &faces[included_faces[i] * 3];
////    Eigen::Vector3d pt1(vertices[face[0] * 3], vertices[face[0] * 3 + 1], vertices[face[0] * 3 + 2]);
////    Eigen::Vector3d pt2(vertices[face[1] * 3], vertices[face[1] * 3 + 1], vertices[face[1] * 3 + 2]);
////    Eigen::Vector3d pt3(vertices[face[2] * 3], vertices[face[2] * 3 + 1], vertices[face[2] * 3 + 2]);
////    Eigen::Vector3d v1 = pt2 - pt1;
////    Eigen::Vector3d v2 = pt3 - pt2;
////    Eigen::Vector3d nv = v1.cross(v2);
////    nv.normalize();
////    normal_vec = normal_vec + nv;
////  }
////  normal_vec.normalize();  // re-normalize should be same as divide by number of normals
//}

//void hmTriHeatPaths::plane_fit(std::shared_ptr<geometrycentral::surface::SurfaceMesh> mesh, size_t vertex_index, Eigen::Vector3d& normal_vec)
//{
////  // this code fits a plane to the vertex, and all its one-ring neighbors to find the normal vector at that vertex
////  // each point (px,py,pz) in a plane satisfies nx*px + ny*py + nz*pz + offset = 0
////  // without loss of generality we may divide by nz to get:
////  // a*px + b*py + 1*c = -pz
////  // When we have N points, this creates N equations
////  // This linear system Ax=b where each row (i) of A is [px_i py_i 1], x is a column vector = [a b c]^t and b is a
////  // column vector b(i) = -z_i

////  // TODO, do something to insure the points are not co-linear, and that there are at least three points

////  // find out how many neighbors
////  double* vertices = distance->surface->vertices;
////  const hmVectorPairSizeTDouble* VN = distance->surface->vertexNeighbors;
////  const hmPairSizeTDouble *neighborsBegin, *neighborsEnd, *currentNeighbor;
////  double* pnt;
////  int nn = VN[vertex_index].size;  // number of neighbors
////  Eigen::MatrixXf A(nn + 1, 3);
////  Eigen::VectorXf b(nn + 1);

////  // add the vertex itself
////  pnt = &(vertices[vertex_index * 3]);
////  A(0, 0) = pnt[0];
////  A(0, 1) = pnt[1];
////  A(0, 2) = 1.0;
////  b(0) = pnt[2];
////  int row = 1;
////  neighborsBegin = VN[vertex_index].entries;
////  neighborsEnd = neighborsBegin + VN[vertex_index].size;
////  for (currentNeighbor = neighborsBegin; currentNeighbor != neighborsEnd; currentNeighbor++)
////  {
////    vertex_index = currentNeighbor->n;
////    pnt = &vertices[vertex_index * 3];
////    A(row, 0) = pnt[0];
////    A(row, 1) = pnt[1];
////    A(row, 2) = 1.0;
////    b(row) = pnt[2];
////    row++;
////  }
////  Eigen::MatrixXf x = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

////  // recall we assumed that nz = 1,
////  double norm = sqrt(x(0, 0) * x(0, 0) + x(1, 0) * x(1, 0) + 1.0);  // can never be zero, isn't that nice
////  normal_vec(0) = x(0, 0) / norm;
////  normal_vec(1) = x(1, 0) / norm;
////  normal_vec(2) = 1.0 / norm;

////  Eigen::Vector3d fn;
////  one_face_normal(distance, vertex_index, fn);

////  // choose normal to match triangle normal convention
////  if (fn.dot(normal_vec) < 0.0)
////    normal_vec *= -1.0;
//}

//void saveVertices(std::vector<int> VI, double* vertices, char* file_name, char* vector_name)
//{
////  FILE* fp;
////  if ((fp = fopen(file_name, "w")))
////  {
////    fprintf(fp, "%s = [\n ", vector_name);
////    for (int i = 0; i < (int)VI->size; i++)
////    {
////      int vi = VI->entries[i] * 3;
////      fprintf(fp, "%lf %lf %lf;\n ", vertices[vi], vertices[vi + 1], vertices[vi + 2]);
////    }
////    fprintf(fp, "];\n");
////    fclose(fp);
////  }
//}

//void showNeighbors(const std::vector<double>* VN, char* msg)
//{
////  printf("showing %s: \n", msg);
////  for (int i = 0; i < (int)VN->size; i++)
////  {
////    printf("VN.entries(%d).n = %ld VN.entries.(%d)x = %lf\n", i, VN->entries[i].n, i, VN->entries[i].x);
////  }
//}


//int max_d_exclude_v(std::vector<double>& d, double v_exclude)
//{
////  // TODO, since most vertices may have no edges, only search those that do
////  int farthest = 0.0;
////  double d_max = 0.0;
////  for (int j = 0; j < (int)d.size(); j++)
////  {
////    if (d[j] != v_exclude && d[j] > d_max)
////    {
////      d_max = d[j];
////      farthest = j;
////    }
////  }
////  return (farthest);
//}
*/

/*
//void hmTriHeatPaths::octave_output_paths(std::string filename, std::shared_ptr<geometrycentral::surface::SurfaceMesh> mesh)
//{
//  char color[6];
//  color[0] = 'r';
//  color[1] = 'g';
//  color[2] = 'b';
//  color[3] = 'c';
//  color[4] = 'm';
//  color[5] = 'k';
//  //  color[6] = 'o';
//  FILE* fp;
//  if (!(fp = fopen(filename.c_str(), "w")))
//  {
//    printf(" cannot open file %s\n", filename.c_str());
//    return;
//  }

//  fprintf(fp, "hold off;\n");
//  bool hold_off = true;
//  int c = 0;
//  for (int i = 0; i < (int)vertex_sequences_.size(); i++)
//  {
//    if (vertex_sequences_[i].size() > 0)
//    {
//      char vector_name[100];
//      double* vertices = distance->surface->vertices;
//      sprintf(vector_name, "C%d", i);
//      fprintf(fp, "%s=[\n", vector_name);
//      for (int j = 0; j < (int)vertex_sequences_[i].size(); j++)
//      {
//        int vi = vertex_sequences_[i][j] * 3;
//        fprintf(fp, "%lf %lf %lf;\n ", vertices[vi], vertices[vi + 1], vertices[vi + 2]);
//      }
//      fprintf(fp, "];\n");
//      fprintf(fp,
//              "plot3(%s(:,1),%s(:,2),%s(:,3), '%c', \"linewidth\", 5);\n",
//              vector_name,
//              vector_name,
//              vector_name,
//              color[c]);
//      c = (c + 1) % 6;
//      if (hold_off)
//      {
//        fprintf(fp, "hold on;\n");
//        hold_off = false;
//      }
//    }
//  }
//  fclose(fp);
//}

//void hmTriHeatPaths::octave_output_sources(std::string filename, std::shared_ptr<geometrycentral::surface::SurfaceMesh> mesh, std::vector<int> S)
//{
//  char color[6];
//  color[0] = 'r';
//  color[1] = 'g';
//  color[2] = 'b';
//  color[3] = 'c';
//  color[4] = 'm';
//  color[5] = 'k';

//  FILE* fp;
//  ///////
//  if (!(fp = fopen(filename.c_str(), "w")))
//  {
//    printf(" cannot open file %s\n", filename.c_str());
//    return;
//  }

//  fprintf(fp, "hold off;\n");
//  bool hold_off = true;
//  int c = 0;
//  char vector_name[100];
//  double* vertices = distance->surface->vertices;
//  sprintf(vector_name, "S");
//  fprintf(fp, "%s=[\n", vector_name);
//  printf("size = %ld\n", S->size);
//  for (int j = 0; j < (int)S->size; j++)
//  {
//    size_t vi = S->entries[j];
//    fprintf(fp, "%lf %lf %lf;\n ", vertices[vi * 3], vertices[vi * 3 + 1], vertices[vi * 3 + 2]);
//  }
//  fprintf(fp, "];\n");
//  fprintf(
//      fp, "plot3(%s(:,1),%s(:,2),%s(:,3), '%c', \"linewidth\", 5);\n", vector_name, vector_name, vector_name, color[c]);
//  c = (c + 1) % 6;
//  if (hold_off)
//  {
//    fprintf(fp, "hold on;\n");
//    hold_off = false;
//  }
//  fclose(fp);
//}
*/
