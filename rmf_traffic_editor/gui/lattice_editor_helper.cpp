// Copyright 2023 @ivrolan - Tractonomy Robotics

#include "lattice_editor_helper.h"


#include <iomanip>
#include <nlohmann/json.hpp>

// for convenience
using json = nlohmann::json;

using std::string;
using std::isnan;

RootLatticeHelper::RootLatticeHelper(Vertex root, int vertex_id, std::string filename_motion_prims, Layer layer, std::vector<lattice::Restriction*> restrictions)
  : Helper(LATTICE_HELPER), root_v_idx(vertex_id), m_prims_path_(filename_motion_prims), layer_(layer)
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

  theta_samples = j_file["theta_samples"].get<std::vector<double>>();
  // create the lattice object
  QPointF root_nav = layer_.transform_layer_to_global(QPointF(root.x, root.y));
  lattice::State r{root_nav.x() / 10, root_nav.y() / 10, lattice::BaseLattice::discretizeAngle(root.theta(), theta_samples)};
  
  if (restrictions.empty()){
    lat_ = new lattice::RootLattice(r, m_prims_);
  } else {
    lat_ = new lattice::RestrictedRootLattice(r, m_prims_, restrictions);
  }
  if (!root.forward_expansion()) {
    lat_->enableReverse();
  } else {
    lat_->disableReverse();
  }
  lat_->toExpand(r);
  
  first_x = r.x;
  first_y = r.y;
}

void RootLatticeHelper::draw(QGraphicsScene * scene, const Level * level_ptr) {

  if (!level_ptr->vertices[root_v_idx].selected)
    return;

  // draw the lattice
  lattice::EdgeList edges;
  double max_cost = 0.0, min_cost = INFINITY;
  // we use the theta of the to_draw State to know if it has been set to other value
  // -1 is not possible in the discretization we have of the angle
  if (to_draw.th == -1) {
    lat_->getAllEdges(edges);
    for (auto& e: edges) {
      //v_marker_arr.markers.push_back(createMarkerFromEdge(e, m_prims_, {2.0f * x, 2.0f * (1 - x), 0}))
      max_cost = std::max<double>(e.cost, max_cost);
      min_cost = std::min<double>(e.cost, min_cost);
    }

  } else {
    lat_->getPath(to_draw, edges);
  }

  // create_scene();
  QPointF root_nav = layer_.transform_layer_to_global(QPointF(level_ptr->vertices[root_v_idx].x,  level_ptr->vertices[root_v_idx].y));

  
  // default color
  QColor color = QColor::fromRgbF(0.0, 0.0, 0.0, 0.5);

  // x = (e.cost - min_cost) / (max_cost - min_cost);
  // {2.0f * x, 2.0f * (1 - x), 0, 0.15}
  double x;
  for (auto& e : edges) {
    // offset in case the vertex is moved
    
    // std::cout << "First is " << first_x << ", " << first_y << std::endl;
    // std::cout << "Root nav / 10 is " << root_nav.x() << ", " << root_nav.y() << std::endl;
    e.start.x += (root_nav.x() / 10 - first_x);
    e.start.y += (root_nav.y() / 10 - first_y);

    if (to_draw.th == -1) {
      x = (e.cost - min_cost) / (max_cost - min_cost);
      color = QColor::fromRgbF((2.0f * x) / 2.0, (2.0f * (1 - x)) / 2.0, 0, 0.5);
    }
    drawLatticeEdge(scene, QBrush(color), e, m_prims_, layer_);
  }

  // setWindowModified(true);
}

bool RootLatticeHelper::resumeExpansion(int n, lattice::EdgeList& edges) {
  max_expand_ += n;
  return lat_->resumeExpansion(n, edges);
}
  