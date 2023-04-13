#include "lattice.hpp"

using lattice::Edge;
using lattice::EdgeList;
using lattice::Adjacency;
using lattice::BaseLattice;
using lattice::RootLattice;
using lattice::RestrictedRootLattice;
using lattice::State;



std::ostream& lattice::operator<<(std::ostream& os,const Edge& e) {
    return os << "Edge:" << std::endl <<
            "- Start: " << e.start <<  std::endl <<
            "- End: " << e.end <<  std::endl <<
            "- Id: " << e.prim_id << std::endl <<
            "- Cost: " << e.cost << std::endl;
}

bool lattice::operator==(const Edge& lhs, const Edge& rhs) {
  if (lhs.start != rhs.start)
    return false;
  if (lhs.end != rhs.end)
    return false;
  if (lhs.prim_id != rhs.prim_id)
    return false;
  // if (lhs.cost != rhs.cost) // it depends on the rest so it shouldnt be important
  //   return false;
  return true;
}

bool lattice::operator!=(const Edge& lhs, const Edge& rhs) {
    return !(lhs==rhs);
}

BaseLattice::~BaseLattice(){map_.clear();}

BaseLattice::BaseLattice(MotionPrimitives& mot_prims) {
  for (MotionPrimitive& m : mot_prims) {
    if (motion_prims_.count(m.start_th) == 0) {
      motion_prims_[m.start_th] = MotionPrimitives{m};
    } else {
      motion_prims_[m.start_th].push_back(m);
    }

    if (rev_motion_prims_.count(m.end_th) == 0) {
      rev_motion_prims_[m.end_th] = MotionPrimitives{m};
    } else {
      rev_motion_prims_[m.end_th].push_back(m);
    }
  }

  // by default non-visited asn visited oint to the forward lists
  non_visited = &fw_non_visited;
  visited = &fw_visited;
}

RootLattice::RootLattice(State& init, MotionPrimitives mot_prims)
: BaseLattice(mot_prims) {
  map_[init] = Adjacency{};
  //states_.push_back(init);
  root = &init;
  backtrack_info_[*root].best_parent = *root;
  backtrack_info_[*root].cost_to_root = 0.0;
}

void test_equal_edges(std::unordered_map<State, EdgeList> map) {
  for (auto& it : map) {
      for (auto ed = (it.second).begin(); ed != (it.second).end() - 1; ++ed) {
        for (auto ed2 = ed + 1; ed2 !=(it.second).end() ; ed2++) {
          if (*ed == *ed2) {
            std::cout << "Found 2 equal edges!" << std::endl;
          }
        }
      }
    }
}

double BaseLattice::getAngleFromIndex(int index, std::vector<double>& th_samples) {
  if (index < th_samples.size()) {
    return th_samples.at(index);
  }

  //  index > th_samples.size()

  if (index <= (((int) th_samples.size()-1) * 2) ) {
    // std::cout << "1: " << index - ((int)th_samples.size()) + 1 << std::endl;
    return th_samples.at(index - ((int)th_samples.size()) + 1 ) + M_PI_2;
  }

  if (index <= (((int) th_samples.size()-1) * 3)) {
    // std::cout << "2: " << index - 2 *(((int)th_samples.size()) - 1) << std::endl;
    return th_samples.at(index - 2 *(((int)th_samples.size()) - 1) ) - M_PI;
  }

  if (index <= (((int) th_samples.size()-1) * 4)) {
    // std::cout << "3: " << index - 3 *(((int)th_samples.size()) - 1)  << std::endl;
    return th_samples.at(index - 3 *(((int)th_samples.size()) - 1) ) - M_PI_2;
  }
  throw std::logic_error("index out of bounds when getting the Angle from the index");
}

int nearestAngle(double ang, std::vector<double>& th_samples) {

  double diff = INFINITY;
  int index = -1;
  for (uint i = 0; i < th_samples.size(); i++) {
    if ( fabs(th_samples.at(i) - ang) < diff) {
      diff = fabs(th_samples.at(i) - ang);
      index = i;
    }
  }

  return index;
}
// assuming that len(th_samples) >= 2, and that starting angle is 0 and end angle is pi/2
int BaseLattice::discretizeAngle(double angle, std::vector<double>& th_samples) {
  // angles defined beqween [-pi/2, pi/2]
  if (angle > M_PI_2) { 
    return 2 * (th_samples.size() - 1) - nearestAngle(M_PI - angle, th_samples);
  }
  if (angle < -M_PI_2) {
    return 2 * (th_samples.size() - 1) + nearestAngle(M_PI + angle, th_samples);
  }

  if (angle < 0) { // [-pi/2, 0]
    return (4 * (th_samples.size() - 1) - nearestAngle(-angle, th_samples))%(4 * (th_samples.size() - 1) );  
  }

  return nearestAngle(angle, th_samples);

}


bool BaseLattice::getNearest(const State& s, State& nearest) {

  // search for the nearest in x and y
  // return it only if it matches the angle
  double d,  min_d = INFINITY;

  for(auto iter = map_.begin(); iter != map_.end(); ++iter){

    d = (iter->first.x - s.x) * (iter->first.x - s.x) +
        (iter->first.y - s.y) * (iter->first.y - s.y);
    
    if (d < min_d) {
      nearest = iter->first;
      min_d = d;
    }
  }
  std::cout << "S: " << s << std::endl;
  std::cout << "Nearest node: " << nearest << std::endl;

  nearest.th = s.th;
  
  std::cout <<"Checking "<< nearest << std::endl;
  //std::cout << "Count of " << nearest << " is " << map_.count(nearest) << std::endl;
  return map_.count(nearest) > 0; // check if it exists a State with thegiven xy and th
}

int RootLattice::getPath(const State& goal, EdgeList& path) {

  State end;

  if(!getNearest(goal, end)) {
    return false; // no path found, because the nearest point is not in the same angle
  }

  // add edges as the backtracing cotinues
  Edge step_back;
  State s1{0,0,0};
  State s2{0,0,0};
  while (end != *root) {

    if (!reverse_enabled_) {
      s1 = backtrack_info_[end].best_parent;
      s2 = end;
    } else {
      s1 = end;
      s2 = backtrack_info_[end].best_parent;
    }

    if (!getEdge(s1, s2, step_back)) {
      std::cerr << "Could not get edge between " << s1 << " and " << s2 << std::endl;
      return false;
    }
    // std::cout << "found edge " << step_back << std::endl;
    std::cout << "current_cost " << backtrack_info_[end].cost_to_root << std::endl;
    if ( backtrack_info_[end].cost_to_root < backtrack_info_[step_back.start].cost_to_root) {
      std::cerr<< "Cost of the child " << end << "is " << backtrack_info_[end].cost_to_root
        << " which is less than the cost of the parent " << step_back.start << " that is "
        << backtrack_info_[step_back.start].cost_to_root << std::endl;
      throw std::logic_error("A path needs to have always a decreasing cost as it is approaching the root");
    }
    path.push_back(step_back);
    end = backtrack_info_[end].best_parent;
  }

  return true;
}

void BaseLattice::getAllEdges(EdgeList& edges){
  
  for(auto iter = map_.begin(); iter != map_.end(); ++iter){
      for (const Edge& e : iter->second.children) {  
        // if not in edges add to edges

        if (! (std::find(edges.begin(), edges.end(), e) != edges.end()) ) {
    
          edges.push_back(e);
        }
      }

      for (const Edge& e : iter->second.parents) {  
        // if not in edges add to edges

        if (! (std::find(edges.begin(), edges.end(), e) != edges.end()) ) {
    
          edges.push_back(e);
        }
      }
  }
}

void RootLattice::updateChildrenCost(State& parent) {

  std::stack<State> parents;
  parents.push(parent);

  State current_parent, child;
  while (!parents.empty()) {
    current_parent = parents.top();
    parents.pop();

    for (Edge& e : map_[current_parent].children) {
      child = e.end;

      if (backtrack_info_[child].cost_to_root >
        backtrack_info_[current_parent].cost_to_root + e.cost)
      {
        // update new cost
        backtrack_info_[child].cost_to_root = backtrack_info_[current_parent].cost_to_root + e.cost;
        backtrack_info_[child].best_parent = current_parent;

        // add to the parents stack to update
        parents.push(child);
      }
    }
  }

}


int BaseLattice::addState(State& to_add, Edge& e) {

  if (e.start != to_add && e.end != to_add) {
    return 1;
  }

  if (e.start == to_add) { // to_add is the parent of a child that already exists

    if (map_.count(e.end) == 0 ) { 
      std::cerr << "the other state is not in the graph" << std::endl;
      return 2;
    }

    map_[e.end].parents.push_back(e);
    map_[e.start].children.push_back(e); 

    
  } else { // e.end == to_add, to_add is the added child
           // to_add was nt in the lattice
    if (map_.count(e.start) == 0 ) {
      std::cerr << "the other state is not in the graph" << std::endl;
      return 2;
    }

    map_[e.end].parents.push_back(e);
    map_[e.start].children.push_back(e); 

   
  }

  return 0;
}

int RootLattice::addState(State& to_add, Edge& e) {

  if (e.start != to_add && e.end != to_add) {
    return 1;
  }

  if (e.start == to_add) { // to_add is the parent of a child that already exists

    if (map_.count(e.end) == 0 ) { 
      std::cerr << "e.end is not in the graph" << std::endl;
      return 2;
    }

    map_[e.end].parents.push_back(e);
    map_[e.start].children.push_back(e); 

    // cost comparison
    if (!reverse_enabled_) {
      if (backtrack_info_[e.end].cost_to_root > e.cost + backtrack_info_[e.start].cost_to_root){
        // change cost and state of e.end
        backtrack_info_[e.end].cost_to_root = e.cost + backtrack_info_[e.start].cost_to_root;
        backtrack_info_[e.end].best_parent = e.start;

        updateChildrenCost(e.end);
      }
    } else {
      if (backtrack_info_[e.start].cost_to_root > e.cost + backtrack_info_[e.end].cost_to_root){
        // change cost and state of e.start
        backtrack_info_[e.start].cost_to_root = e.cost + backtrack_info_[e.end].cost_to_root;
        backtrack_info_[e.start].best_parent = e.end;

        updateChildrenCost(e.start);
      }
    }
  } else { // e.end == to_add, to_add is the added child
           // to_add was nt in the lattice
    if (map_.count(e.start) == 0  && !reverse_enabled_) {
      std::cerr << "e.start is not in the graph" << std::endl;
      return 2;
    }

    map_[e.end].parents.push_back(e);
    map_[e.start].children.push_back(e); 

    State end = !reverse_enabled_ ?  e.end: e.start;
    State start = !reverse_enabled_ ?  e.start : e.end;

    if (backtrack_info_.count(end) == 0) {
      backtrack_info_[end].best_parent = start;
      backtrack_info_[end].cost_to_root = e.cost + backtrack_info_[start].cost_to_root;
    } else {
      if (backtrack_info_[end].cost_to_root > e.cost + backtrack_info_[start].cost_to_root){
        // change cost and state of end
        backtrack_info_[end].cost_to_root = e.cost + backtrack_info_[start].cost_to_root;
        backtrack_info_[end].best_parent = start;

        updateChildrenCost(end);
      }
    }
  }

  return 0;
}

void BaseLattice::clear() {
  map_.clear();
}

void RootLattice::clear() {
  BaseLattice::clear();
  map_[*root] = Adjacency{EdgeList{}, EdgeList{}}; // add root to the lattice
}

Adjacency* BaseLattice::getAdjacency(State& s) {
  return &(map_[s]);
}

State BaseLattice::createNewState(const State& start,  MotionPrimitive m ) {
  State s;

  s.x = start.x + m.poses.back().at(0);

  s.y = start.y + m.poses.back().at(1);

  s.th = m.end_th;

  return s;
}


int BaseLattice::expandState(State& s, std::vector<State>& expanded, EdgeList& edges_expanded) {
  // first, retrieve the possible next states by the primitives according to the angle of the state

  MotionPrimitives possible_prims;
  
  if (reverse_enabled_) {
    possible_prims = rev_motion_prims_[s.th];
  } else {
    possible_prims = motion_prims_[s.th];
  }

  Edge e;
  //State expanded_s;
  //std::vector<State> expanded;
  for (MotionPrimitive& m : possible_prims) {
    e = createEdge(s, m); 
  
    addState(e.end, e);
    if (reverse_enabled_) {
      expanded.push_back(e.start);
    } else {
      expanded.push_back(e.end);
    }
    edges_expanded.push_back(e);

  }

  return edges_expanded.size();
}

int BaseLattice::expandState(State& s, std::vector<State>& expanded) {
  EdgeList foo;
  return expandState(s, expanded, foo);
}

bool BaseLattice::getEdge(State& s1, State& s2, Edge& e_found) {
  // if there is edge between s1 and s2, s1 being the parent of s2
  // s1 -> s2

  for (Edge& e : map_[s1].children) {
    if ((e.start == s1 && e.end == s2)) {
      e_found = e;
      return true;
    }
  }

  return false;

}

int BaseLattice::resumeExpansion(int max_exp, EdgeList& new_edges) {
  int n = 0;
  State current;
  std::vector<State> expanded;
  EdgeList edges_expanded;
  // std::cout << max_exp << std::endl;
  // std::cout << non_visited->empty()  << std::endl;

  while (!non_visited->empty() && (n < max_exp || max_exp == 0)) {
    current = non_visited->front();
    non_visited->pop_front();
    n++;
    //std::cout << "N is " << n  << " and max_Exp is" << max_exp << std::endl ;    

    expanded.clear();
    edges_expanded.clear();
    this->expandState(current, expanded, edges_expanded);

    for ( uint i = 0; i < expanded.size(); i++) {
      new_edges.push_back(edges_expanded.at(i));
      if (std::find(visited->begin(), visited->end(), expanded.at(i)) == visited->end()) {
        non_visited->push_back(expanded.at(i));
        visited->push_back(expanded.at(i)); 
      }
    }
  }
  return !non_visited->empty();
}

int BaseLattice::resumeExpansion(int max_exp) {
  int n = 0;
  State current;
  std::vector<State> expanded;

  // std::cout << max_exp << std::endl;
  // std::cout << non_visited->empty()  << std::endl;

  while (!non_visited->empty() && (n < max_exp || max_exp == 0)) {

    current = non_visited->front();
    n++;

    non_visited->pop_front();
    expanded.clear();
    // edges_expanded.clear();
    this->expandState(current, expanded);

    for ( State& s : expanded) {

      if (! (std::find(visited->begin(), visited->end(), s) != visited->end()) ) {
   
        non_visited->push_back(s);
        visited->push_back(s); 
        //visited->insert(s);
      }
    }
  }
  return n > 0;
}

MotionPrimitive BaseLattice::reverseMotion(MotionPrimitive& m) {

  MotionPrimitive reversed = m;

  // the pose used by createNewState() is the one at the back()
  // we need to reverse the order too then
  for (size_t i = 0; i < reversed.poses.size(); i++) {
    reversed.poses.at(i).at(0) = m.poses.at(reversed.poses.size() - 1 - i).at(0) - reversed.poses.back().at(0);
    reversed.poses.at(i).at(1) = m.poses.at(reversed.poses.size() - 1 - i).at(1) -reversed.poses.back().at(1);
  }

  reversed.start_th = m.end_th;
  reversed.end_th = m.start_th;

  return reversed;
}

void BaseLattice::enableReverse() {
  // std::cout << "Enabled REVERSED" << std::endl;
  reverse_enabled_ = true;
  non_visited = &(rev_non_visited);
  visited = &(rev_visited);
}

void BaseLattice::disableReverse() {
  // std::cout << "disabled REVERSED" << std::endl;
  reverse_enabled_ = false;
  non_visited = &(fw_non_visited);
  visited = &(fw_visited);
}

Edge BaseLattice::createEdge(State& s, MotionPrimitive& m) {
  
  Edge e;
  e.cost = m.cost;
  e.prim_id = m.id;
  
  if (reverse_enabled_) {
    e.end = s;
    e.start = createNewState(s, reverseMotion(m));
  } else {
    e.start = s;
    e.end = createNewState(s, m);
  }

  return e;
}

void BaseLattice::toExpand(State s) {
  non_visited->push_back(s);
  visited->push_back(s);
}

int BaseLattice::expandLattice(int max_exp) {
  return resumeExpansion(max_exp);
}

int RootLattice::expandLattice(int max_exp) {

  // start from the root
  std::cout << max_exp << std::endl;
  if (root == nullptr) {
    return 1;
  }

  //std::deque<State> non_visited;
  non_visited->push_back(*root);
  
  //std::vector<State> visited;
  visited->push_back(*root);
  
  return BaseLattice::expandLattice(max_exp);
}

std::ostream& lattice::operator<<(std::ostream& os,const BaseLattice& l) {

  int counter = 0;
  for(auto iter = l.map_.begin(); iter != l.map_.end(); ++iter){
      State cur = iter->first; // pointer to State*
      
      os << "State " << counter <<  ": " << cur << std::endl;
      os << "EdgeList (Size: " << (l.map_).at(cur).children.size() << " children" <<
          (l.map_).at(cur).parents.size() << " parents):" << std::endl;
        
      os << "Children:" << std::endl;
      for (const Edge& e : (l.map_).at(cur).children) {   // we have to use at(), as Lattice is const
        //os << "   Edge:" << std::endl;

        os << "\t" << e << std::endl;
      }
      os << "Parents:" << std::endl;
      for (const Edge& e : (l.map_).at(cur).parents) {   // we have to use at(), as Lattice is const
        //os << "   Edge:" << std::endl;

        os << "\t" << e << std::endl;
      }

      counter++;
  }
  return os;
}

RestrictedRootLattice::RestrictedRootLattice(State& init, MotionPrimitives& mot_prims,  std::vector<Restriction*> restrictions)
  : RootLattice(init, mot_prims), restrictions_(restrictions) {}

RestrictedRootLattice::RestrictedRootLattice(RootLattice* base, std::vector<Restriction*> restrictions)
  : RootLattice(*(base->root), {}), restrictions_(restrictions) {
    if ( base->isReversed()){
      rev_motion_prims_ = base->getMotionPrims();
      base->disableReverse();
      motion_prims_ = base->getMotionPrims();
      base->enableReverse();
    } else {
      motion_prims_ = base->getMotionPrims();
      base->enableReverse();
      rev_motion_prims_ = base->getMotionPrims();
      base->disableReverse();
    }
}

int RestrictedRootLattice::expandState(State& s, ::std::vector<State>& expanded, EdgeList& edges_expanded) {
  MotionPrimitives possible_prims;
  
  if (reverse_enabled_) {
    possible_prims = rev_motion_prims_[s.th];
  } else {
    possible_prims = motion_prims_[s.th];
  }

  Edge e;
  Point to_check;
  bool valid = true;
  //State expanded_s;
  //std::vector<State> expanded;
  for (MotionPrimitive& m : possible_prims) {
    e = createEdge(s, m); 

    
    if (reverse_enabled_) {
      to_check.x = e.start.x;
      to_check.y = e.start.y;
    } else {
      to_check.x = e.end.x;
      to_check.y = e.end.y;
    }
    
    for (auto restrict_ptr : restrictions_) {
      if (!restrict_ptr->check(to_check)) {
        valid = false;
      }
    }

    if (!valid)
      continue;


    addState(e.end, e);
    if (reverse_enabled_) {
      expanded.push_back(e.start);
    } else {
      expanded.push_back(e.end);
    }
    edges_expanded.push_back(e);

  }

  return edges_expanded.size();
}

  
