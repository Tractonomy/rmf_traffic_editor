#ifndef LATTICE_DRAW_UTILS_H
#define LATTICE_DRAW_UTILS_H

#include "ceres/ceres.h"
#include <QGraphicsOpacityEffect>
#include <QGraphicsPixmapItem>
#include <QGraphicsScene>
#include <QImage>
#include <QImageReader>

#include "layer.h"
#include "lattice.hpp"

void drawLatticeEdge( QGraphicsScene* scene, QBrush brush,
  const lattice::Edge& edge, MotionPrimitives& m_prims, Layer& tf_layer);


#endif // LATTICE_DRAW_UTILS_H