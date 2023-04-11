#ifndef MOTION_PRIMITIVE
#define MOTION_PRIMITIVE

#include <vector>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/access.hpp>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/vector.hpp>

struct MotionPrimitive
{
  int id;
  int start_th;
  int end_th;
  double cost;
  std::vector<std::vector<double>> poses;
};

typedef std::vector<MotionPrimitive> MotionPrimitives;


namespace boost {
namespace serialization {
  template<class Archive>
  void serialize(Archive & ar, MotionPrimitive & m, const unsigned int version) {
    ar & m.id;
    ar & m.start_th;
    ar & m.end_th;
    ar & m.cost;
    ar & m.poses; // vector of vector of double
  }
} // namespace serialization
} // namespace boost

#endif // MOTION_PRIMITIVE