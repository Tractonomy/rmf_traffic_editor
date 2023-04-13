#ifndef LATTICE_HPP
#define LATTICE_HPP

#include <sstream>
#include <iostream>
#include <ostream>
#include <vector>
#include <set>
#include <stack>
#include <stdexcept>
#include <unordered_map>
#include <deque>
#include <algorithm>

#include "state.hpp"
#include "motion_primitive.hpp"
#include "restriction.hpp"

#include <nlohmann/json.hpp>

// for convenience
using json = nlohmann::json;

namespace lattice {

struct Edge {

  State start;
  State end;
  int prim_id; // we dont need the poses to build the path
  double cost;

  friend ::std::ostream& operator<<(::std::ostream& os,const Edge& e);

  friend bool operator==(const Edge& lhs, const Edge& rhs);
  friend bool operator!=(const Edge& lhs, const Edge& rhs);
};

::std::ostream& operator<<(::std::ostream& os,const Edge& e);
bool operator==(const Edge& lhs, const Edge& rhs);
bool operator!=(const Edge& lhs, const Edge& rhs);

typedef ::std::vector<Edge> EdgeList;

struct Adjacency {
  EdgeList children;
  EdgeList parents;
};

struct BacktrackInfo {
  double cost_to_root;
  State best_parent;
};

// TODO: this should use templates to be more general(?)

class BaseLattice {
public:
  virtual ~BaseLattice();
  
  // define how the lattice states are created with MotionPimitives
  BaseLattice(MotionPrimitives& mot_prims);
  
  // edge e should conect to_add to an existing State in the map 
  virtual int addState(State& to_add, Edge& e);
  bool getNearest(const State& s, State& nearest);
  Adjacency* getAdjacency(State& s);
  void getAllEdges(EdgeList& edges);
  bool getEdge(State& s1, State& s2, Edge& e_found);
  virtual int expandState(State& s, ::std::vector<State>& expanded, EdgeList& edges_expanded);
  virtual int expandState(State& s, ::std::vector<State>& expanded);
  
  // add s to the visited and non_visited lists to be expanded later
  void toExpand(State s);
  
  // as we don't have the root, it is the same than resumeExpansion()
  virtual int expandLattice(int max_exp = 0);

  // takes the viisted and non_visited lists and expand the rest of states
  int resumeExpansion(int max_exp);
  // takes the viisted and non_visited lists and expand the rest of states
  // append to edges the expanded nodes
  int resumeExpansion(int max_exp, EdgeList& edges);
  virtual void clear();

  // assuming that len(th_samples) >= 2, and that starting angle is 0 and end angle is pi/2
  static int discretizeAngle(double angle, ::std::vector<double>& th_samples);
  
  static double getAngleFromIndex(int index, ::std::vector<double>& th_samples);

  friend ::std::ostream& operator<<(::std::ostream& os,const BaseLattice& l);

  ::std::map<State, Adjacency> map_; // changed to public for debugging

  void enableReverse();
  void disableReverse();
  inline bool isReversed() {return reverse_enabled_;}

  inline ::std::unordered_map<int, MotionPrimitives> getMotionPrims() {return  reverse_enabled_ ?  rev_motion_prims_: motion_prims_;}

  ::std::deque<State>* non_visited; // public for debugging
  
  template<class Archive>
  friend void ::boost::serialization::serialize(Archive & ar, BaseLattice & l, const unsigned int version);

protected:

  State createNewState(const State& start,  MotionPrimitive m );
  Edge createEdge(State& s, MotionPrimitive& m);

  MotionPrimitive reverseMotion(MotionPrimitive& m);

  // int expandStateForward(State& s, ::std::vector<State>& expanded, EdgeList& edges_expanded);
  // int expandStateReverse(State& s, ::std::vector<State>& expanded, EdgeList& edges_expanded);
  // links a list of primitives to their initial angle
  ::std::unordered_map<int, MotionPrimitives> motion_prims_;
  ::std::unordered_map<int, MotionPrimitives> rev_motion_prims_;

  ::std::vector<State>* visited;

  ::std::deque<State> rev_non_visited;
  ::std::vector<State> rev_visited;

  ::std::deque<State> fw_non_visited;
  ::std::vector<State> fw_visited;

  bool reverse_enabled_ = false;
};

::std::ostream& operator<<(::std::ostream& os,const BaseLattice& l);


class RootLattice : public BaseLattice
{
 public:
  // define how the lattice states are created with MotionPimitives
  RootLattice(State& init, MotionPrimitives mot_prims);

  // edge e should conect to_add to an existing State in the map 
  int addState(State& to_add, Edge& e) override;
  int getPath(const State& goal, EdgeList& path);
  int expandLattice(int max_exp) override;
  void clear() override;

  template<class Archive>
  friend void ::boost::serialization::serialize(Archive & ar, RootLattice & s, const unsigned int version);


  State* root = nullptr;
 protected:
  void updateChildrenCost(State& parent);


  ::std::unordered_map<State, BacktrackInfo> backtrack_info_;

  // we can iterate over the map, a vector of States is not necessary
  // ::std::vector<State> states_;
};

class RestrictedRootLattice : public RootLattice
{
public:
  RestrictedRootLattice(State& init, MotionPrimitives& mot_prims,  std::vector<Restriction*> restrictions);
  RestrictedRootLattice(RootLattice* base, std::vector<Restriction*> restrictions);

  // check if all the restrictions are satisfied
  int expandState(State& s, ::std::vector<State>& expanded, EdgeList& edges_expanded) override;

protected:
  std::vector<Restriction*> restrictions_;
};

} // namespace lattice

// serialization of our data structures to save/load the lattice
namespace boost {
namespace serialization {

  // ----- EDGE ------

  template<class Archive>
  void serialize(Archive & ar, lattice::Edge & e, const unsigned int version) {
    ar & e.start;
    ar & e.prim_id;
    ar & e.end;
    ar & e.cost;
  }

  // ---- ADJACENCY LIST ----

  template<class Archive>
  void serialize(Archive & ar, lattice::Adjacency & adj, const unsigned int version) {
    ar & adj.children;
    ar & adj.parents;
  }

  // ---- BACKTRACK INFO ----
  
  template<class Archive>
  void serialize(Archive & ar, lattice::BacktrackInfo & backtrck, const unsigned int version) {
    ar & backtrck.best_parent;
    ar & backtrck.cost_to_root;
  }

  // ---- BASE LATTICE ----

  template<class Archive>
  void serialize(Archive & ar, lattice::BaseLattice & l, const unsigned int version) {
    ar & l.map_;
    ar & l.rev_non_visited;
    ar & l.rev_visited;
    ar & l.fw_non_visited;
    ar & l.fw_visited;
    ar & l.non_visited;
    ar & l.visited;

    ar & l.reverse_enabled_;
    ar & l.rev_motion_prims_;
    ar & l.motion_prims_;
  }

  // ---- ROOT LATTICE ----

  template<class Archive>
  void serialize(Archive & ar, lattice::RootLattice & l, const unsigned int version) {
    ar & base_object<lattice::BaseLattice>(l);
    ar & l.backtrack_info_;
    ar & l.root;
  }

} // namespace serialization
} // namespace boost
#endif // LATTICE_HPP