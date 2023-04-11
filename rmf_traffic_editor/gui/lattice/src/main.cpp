#include "lattice.hpp"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <string>
#include <chrono>


#include <nlohmann/json.hpp>

// for convenience
using json = nlohmann::json;

void parse_json(json& j_file, MotionPrimitives& m_prims) {

  for (auto& prim : j_file["primitives"]) {
    
    MotionPrimitive mp_to_add;
    
    mp_to_add.id = prim["id"].get<int>();
    mp_to_add.cost = prim["cost"].get<double>();
    mp_to_add.start_th = prim["start"].get<int>();
    mp_to_add.end_th = prim["end"].get<int>();

    mp_to_add.poses = prim["trajectory"].get<std::vector<std::vector<double>>>();

    m_prims.push_back(mp_to_add);
  }


}

int main() {

  State root;
  root.x = 0.0;
  root.y = 0.0;
  root.th = 4;

  // create the primitives from the json file

  std::string json_filename = "../primitive_generation/control_set.json";
  
  MotionPrimitives control_set;

  std::ifstream myfile(json_filename);
  json json_data;
  if ( myfile.is_open() ) {
    json_data = json::parse(myfile);
  }

  parse_json(json_data, control_set);

  std::cout << control_set.size() << std::endl;

  Lattice lat(root, control_set);

  // Start measuring time
  auto begin = std::chrono::high_resolution_clock::now();
  std::vector<State> exp;
  lat.expandState(root, exp);
  // Stop measuring time and calculate the elapsed time
  auto end = std::chrono::high_resolution_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);

  //std::cout << lat << std::endl;
  std::cout << "Elapsed " << elapsed.count() * 1e-9 << "sec for size " << control_set.size() << std::endl;
  
  std::cout << "Expanded list is: " << std::endl;

  for (auto e : exp) {
    std::cout << e << std::endl;
  } 

  std::cout << "Test Lattice Expansion" << std::endl;
  lat.clear();
  begin  = std::chrono::high_resolution_clock::now();
  std::cout << lat.expandLattice(100) << std::endl;
  end = std::chrono::high_resolution_clock::now();
  elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);

  std::cout << lat << std::endl;

  std::cout << "Elapsed " << elapsed.count() * 1e-9 << "sec for size " << control_set.size() << std::endl;
  
  return 0;
}