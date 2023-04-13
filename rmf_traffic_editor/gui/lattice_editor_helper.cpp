// Copyright 2023 @ivrolan - Tractonomy Robotics

#include "lattice_editor_helper.h"


#include <iomanip>
#include <nlohmann/json.hpp>

// for convenience
using json = nlohmann::json;

using std::string;
using std::isnan;

RootLatticeHelper::RootLatticeHelper(Vertex root, int vertex_id, std::string filename_motion_prims, Layer layer)
  : Helper(LATTICE_HELPER), root_v_idx_(vertex_id), m_prims_path_(filename_motion_prims), layer_(layer)
{
  
  json j_file;
  std::ifstream myfile(filename_motion_prims);
  if ( myfile.is_open() ) {
    j_file = json::parse(myfile);
  } else {
    std::cerr << "Could not open " << filename_motion_prims << std::endl;
    return;
  }

  //MotionPrimitives m_prims;

  parseJson2MotPrims(j_file, m_prims_);

  std::vector<double> theta_samples = j_file["theta_samples"].get<std::vector<double>>();
  // create the lattice object
  QPointF root_nav = layer_.transform_layer_to_global(QPointF(root.x, root.y));
  lattice::State r{root_nav.x() / 10, root_nav.y() / 10, lattice::BaseLattice::discretizeAngle(root.theta(), theta_samples)};
  lat_ =  new lattice::RootLattice(r, m_prims_);
  lat_->enableReverse();
  lat_->toExpand(r);
  
  first_x = r.x;
  first_y = r.y;
}

void RootLatticeHelper::draw(QGraphicsScene * scene, const Level * level_ptr) {

  if (!level_ptr->vertices[root_v_idx_].selected)
    return;

  // draw the lattice
  lattice::EdgeList edges;
  lat_->getAllEdges(edges);

  // create_scene();
  QPointF root_nav = layer_.transform_layer_to_global(QPointF(level_ptr->vertices[root_v_idx_].x,  level_ptr->vertices[root_v_idx_].y));;

  // show the lattice only when the root is selected??

  double max_cost = 0.0, min_cost = INFINITY;
  for (auto& e: edges) {
    //v_marker_arr.markers.push_back(createMarkerFromEdge(e, m_prims_, {2.0f * x, 2.0f * (1 - x), 0}))
    max_cost = std::max<double>(e.cost, max_cost);
    min_cost = std::min<double>(e.cost, min_cost);
  }
  // x = (e.cost - min_cost) / (max_cost - min_cost);
  // {2.0f * x, 2.0f * (1 - x), 0, 0.15}
  double x;
  for (auto& e : edges) {
    // offset in case the vertex is moved
    
    // std::cout << "First is " << first_x << ", " << first_y << std::endl;
    // std::cout << "Root nav / 10 is " << root_nav.x() << ", " << root_nav.y() << std::endl;
    e.start.x += (root_nav.x() / 10 - first_x);
    e.start.y += (root_nav.y() / 10 - first_y);


    x = (e.cost - min_cost) / (max_cost - min_cost);
    //rmf_transf = layer_.transform_global_to_layer(QPointF(e.start.x * 10, e.start.y * 10));
    // undo_stack.push(
    // new AddVertexCommand(
    //   &building,
    //   level_idx,
    //   rmf_transf.x(),
    //   rmf_transf.y()));

    drawLatticeEdge(scene, QBrush(QColor::fromRgbF((2.0f * x) / 2.0, (2.0f * (1 - x)) / 2.0, 0, 0.5)), e, m_prims_, layer_);
  }

  // setWindowModified(true);
}

bool RootLatticeHelper::resumeExpansion(int n, lattice::EdgeList& edges) {
  max_expand_ += n;
  return lat_->resumeExpansion(n, edges);
}
  