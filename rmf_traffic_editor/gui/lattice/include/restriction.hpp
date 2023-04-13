#ifndef RESTRICTION_HPP
#define RESTRICTION_HPP

#include <vector>


namespace lattice
{

struct Point{
  double x,y;
};

struct Line {
    Point p1, p2;
};

class Restriction
{
  public:
    // takes point to check if the point satisfies the restriction
    virtual bool check(Point point) = 0;
};

class SimpleRegionCheck : public Restriction
{
  public:
    // set of points that define a region
    SimpleRegionCheck(std::vector<Point> region);

    bool check(Point point) override;

  private:
    std::vector<Point> region_;
};
} // namespace lattice


#endif // RESTRICTION_HPP