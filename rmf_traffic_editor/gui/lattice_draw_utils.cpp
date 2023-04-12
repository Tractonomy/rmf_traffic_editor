#include "lattice_draw_utils.h"

void drawLatticeEdge( QGraphicsScene* scene, QBrush brush,
  const lattice::Edge& e, MotionPrimitives& m_prims, Layer& tf_layer) {

  MotionPrimitive selected_prim;
  bool found = false;
  for (const auto& m : m_prims) {
    if ( m.id == e.prim_id){
      selected_prim = m;
      found = true;
      break;
    }
  }

  if (!found) {
    std::cout << "Edge prim_id not found in motion primitives given" << std::endl;
    return;
  }
  
  QPen pen( brush,
      5.0);

  
  std::vector<double> start, end;
  QPointF tf_start, tf_end;
  for (uint i = 1; i < selected_prim.poses.size(); i++) {
    start = selected_prim.poses.at(i-1);
    end = selected_prim.poses.at(i);

    // convert the coords according to layer

    tf_start = tf_layer.transform_global_to_layer(QPointF((start.at(0) + e.start.x) * 10, (start.at(1) + e.start.y) * 10));
    tf_end = tf_layer.transform_global_to_layer(QPointF((end.at(0) + e.start.x) * 10, (end.at(1) + e.start.y) * 10));
    
    // draw a line in the scene between pose i-1 and i
    QGraphicsLineItem * line = scene->addLine(tf_start.x(), tf_start.y(),
      tf_end.x(),tf_end.y(), pen);

    //line->setZValue(20.0);
  }
}