#ifndef STRUCT_DEFS_HPP
#define STRUCT_DEFS_HPP

struct Particle
{
  double x, y;
};

struct HumanData
{
  HumanData(const double x, const double y)
    : x{x}, y{y}, vx{0.0}, vy{0.0}, ang{0.0}
  {}

  HumanData(const double x, const double y, const double vx, const double vy, const double ang)
    : x{x}, y{y}, vx{vx}, vy{vy}, ang{ang}
  {}

  double x, y, vx, vy, ang;
};

#endif /* STRUCT_DEFS_HPP */

