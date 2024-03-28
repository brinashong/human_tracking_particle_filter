#ifndef STRUCT_DEFS_HPP
#define STRUCT_DEFS_HPP

struct Particle
{
  double x, y;
};

struct HumanData
{
  HumanData(const double x, const double y) //, const double vx, const double vy)
    : x{x}, y{y}//, vx{vx}, vy{vy}
  {}

  double x, y; //, vx, vy;
};

#endif /* STRUCT_DEFS_HPP */

