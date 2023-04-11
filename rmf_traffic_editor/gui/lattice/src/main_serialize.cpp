// Copyright @ivrolan Tractonomy Robotics
// Main program to test serialization of clases and structs needed for serialization of LAttice members
#include "state.hpp"
#include "motion_primitive.hpp"
#include "lattice.hpp"

#include <cstddef> // NULL
#include <iomanip>
#include <iostream>
#include <fstream>
#include <string>
#include <boost/archive/tmpdir.hpp>

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

// compile with -lboost_serialization

int main(int argc, char * argv[]) {

  // ---- TEST STATE SERIALIZATION ----
  State s{1.0, 2.0, 4};

  std::ofstream state_ofs("state_serialized.txt");
  boost::archive::text_oarchive state_oa(state_ofs, 0);
  state_oa << s;

  // ---- TEST MOTION_PRIM SERIALIZATION ----
  
  MotionPrimitive m{1, 2, 3, 4.0, {{5.0, 6.0}, {7.0, 8.0}}};

  std::ofstream motion_ofs("motion_serialized.txt");
  boost::archive::text_oarchive motion_oa(motion_ofs, 0);
  motion_oa << m;


  MotionPrimitive m2{10, 20, 30, 40.0, {{50.0, 60.0}, {70.0, 80.0}}};

  MotionPrimitives ms{m,m2};
  std::ofstream motions_ofs("motions_serialized.txt");
  boost::archive::text_oarchive motions_oa(motions_ofs, 0);
  motions_oa << ms;

  BaseLattice l(ms);
  RootLattice r(s, ms);

  l.expandLattice(5);
  r.expandLattice(5);

  std::ofstream lattice_ofs("base_lattice_serialized.txt");
  boost::archive::text_oarchive lattice_oa(lattice_ofs, 0);
  lattice_oa << l;

  std::ofstream root_lattice_ofs("root_lattice_serialized.txt");
  boost::archive::text_oarchive root_lattice_oa(root_lattice_ofs, 0);
  root_lattice_oa << r;

  return 0;
}