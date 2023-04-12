// Copyright 2023 @ivrolan - Tractonomy Robotics

#ifndef HELPER_H
#define HELPER_H

#include <QGraphicsScene>

//#include "level.h"
class Level;

// class to visualize extra info on the editor
class Helper {

public:
  Helper(int id) : id_(id) {};

  enum HelperID {
    UNDEFINED = 0,
    LATTICE_HELPER
  };
  
  virtual void draw(QGraphicsScene * scene, const Level * level_ptr) = 0;

protected:
  int id_;

};

#endif
