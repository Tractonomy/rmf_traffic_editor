#ifndef LATTICE_HELPERS_HPP
#define LATTICE_HELPERS_HPP

#include "lattice.hpp"
#include "motion_primitive.hpp"

#include <sstream>
#include <fstream>
#include <iomanip>
#include <nlohmann/json.hpp>

// for convenience
using json = nlohmann::json;


// we can do getDiscAngle just with
// BaseLattice::discretizeAngle(angle, j_file["theta_samples"].get<std::vector<double>>())

// int getDiscAngle(json& j_file, double angle) {
//   std::vector<double> th_samples = j_file["theta_samples"].get<std::vector<double>>();

//   return BaseLattice::discretizeAngle(angle, th_samples);
// }

void parseJson2MotPrims(json& j_file, MotionPrimitives& m_prims);

#endif // LATTICE_HELPERS_HPP