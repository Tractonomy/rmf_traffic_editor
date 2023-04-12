#include "lattice_helpers.hpp"


void parseJson2MotPrims(json& j_file, MotionPrimitives& m_prims) {

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