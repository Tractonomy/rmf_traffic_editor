#include "lattice/include/state.hpp"
#include <math.h>

unsigned int my_float_hash( float f)
{
    unsigned int ui;
    memcpy( &ui, &f, sizeof( float ) );
    return ui & 0xfffc0000; 
}

bool lattice::operator==(const lattice::State& lhs, const lattice::State& rhs) {

  if (fabs(lhs.x - rhs.x) > TOLERANCE)
    return false;
  if (fabs(lhs.y - rhs.y) > TOLERANCE)
    return false;
  return (lhs.th ==  rhs.th);
}

bool lattice::operator<(const lattice::State& lhs, const lattice::State& rhs) {

  if (rhs.x - lhs.x > TOLERANCE) {
    return true;
  } else if (rhs.x - lhs.x < -TOLERANCE)
  {
    return false;
  }

  if (rhs.y - lhs.y > TOLERANCE) {
    return true;
  } else if (rhs.y - lhs.y < -TOLERANCE)
  {
    return false;
  }
  
  if (lhs.th <  rhs.th) {
    return true;
  } else if (lhs.th > rhs.th)
  {
    return false;
  }
  return false; // they are equal
}

std::ostream& lattice::operator<<(std::ostream& os,const lattice::State& s) {
  return os << "[ " << s.x << ", " << s.y <<", " << s.th <<"]";
}