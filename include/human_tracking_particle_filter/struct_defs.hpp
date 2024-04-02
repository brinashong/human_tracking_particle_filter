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

  HumanData(const double x, const double y, const double vx, const double vy, const double ang,
            const double ox, const double oy, const double oz, const double ow)
    : x{x}, y{y}, vx{vx}, vy{vy}, ang{ang}
    , ox{ox}, oy{oy}, oz{oz}, ow{ow}
  {}

  double x, y, vx, vy, ang;
  double ox, oy, oz, ow;
};

#endif /* STRUCT_DEFS_HPP */

