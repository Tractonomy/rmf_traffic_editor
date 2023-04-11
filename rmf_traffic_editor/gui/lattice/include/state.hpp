#ifndef STATE
#define STATE

#include <ostream>
#include <cstring>


#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/access.hpp>

#include <boost/serialization/serialization.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/unordered_map.hpp>
#include <boost/serialization/deque.hpp>


#define TOLERANCE 0.05


unsigned int my_float_hash( float f);

namespace lattice {

struct State
{
  double x;
  double y;
  int th; // discretized angle

  friend bool operator==(const State& lhs, const State& rhs);
  friend bool operator<(const State& lhs, const State& rhs);
  friend inline bool operator!=(const State& lhs, const State& rhs) { return !(lhs == rhs); }
  friend ::std::ostream& operator<<(::std::ostream& os,const State& e);


  //friend class boost::serialization::access;

  // template<class Archive>
  // friend void boost::serialization::serialize(Archive & ar, State & s, const unsigned int version);
};

namespace boost {
namespace serialization {
  template<class Archive>
  void serialize(Archive & ar, State & s, const unsigned int version) {
    ar & s.x;
    ar & s.y;
    ar & s.th;
  }
} // namespace serialization
} // namespace boost

} // namespace lattice

namespace std {
template <>
struct hash<lattice::State> {
  auto operator()(const lattice::State &s) const -> size_t {
    return my_float_hash(s.x) ^ (my_float_hash(s.y) >> 14) ^ s.th;
    // shift to the right 14 bits to leave space for the first hash
  }
};

}  // namespace std
#endif // STATE
