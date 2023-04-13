// Copyright 2023 @ivrolan - Tractonomy Robotics

#ifndef LATTICE_EDITOR_HELPER_H
#define LATTICE_EDITOR_HELPER_H

#include "helper.h"
#include "lattice.hpp"
#include "lattice_helpers.hpp"

#include <QtWidgets>

#include <QInputDialog>
#include <QLabel>
#include <QListWidget>
#include <QToolBar>

#include <QGraphicsItem>
#include <QGraphicsEllipseItem>
#include <QGraphicsPixmapItem>
#include <QGraphicsPolygonItem>
#include <QGraphicsScene>
#include <QMainWindow>
#include <QSettings>
#include <QUndoStack>

#include "add_param_dialog.h"
#include "building_dialog.h"
#include "editor.h"
#include "layer_dialog.h"
#include "layer_table.h"
#include "level_dialog.h"
#include "level_table.h"
#include "lift_table.h"
#include "map_view.h"
#include "model_dialog.h"
#include "preferences_dialog.h"
#include "preferences_keys.h"
#include "traffic_table.h"
#include "ui_new_building_dialog.h"
#include "ui_transform_dialog.h"

#include "vertex.h"

#include "lattice.hpp"
#include "lattice_helpers.hpp"

class RootLatticeHelper : public Helper
{
 public:
  RootLatticeHelper(Vertex root, int vertex_id, std::string filename_motion_prims, Layer layer, std::vector<lattice::Restriction*> restrictions = {});
  inline ~RootLatticeHelper(){ delete lat_;};
  
  // maybe we should make RootLattice* public
  // and access this methods drectly through the pointer

  bool resumeExpansion(int n, lattice::EdgeList& edges_expanded);
  bool computePath(lattice::State start, lattice::State goal);
  void draw(QGraphicsScene * scene, const Level * level_ptr) override;
  
 private:
  lattice::RootLattice* lat_;
  int root_v_idx_;
  std::string m_prims_path_;
  int max_expand_ = 0;
  Layer layer_;
  MotionPrimitives m_prims_;

  double first_x;
  double first_y;
};

#endif
