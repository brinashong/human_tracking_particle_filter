#ifndef STRUCT_DEFS_HPP
#define STRUCT_DEFS_HPP

struct Particle
{
  double x, y, w;
};

struct HumanData
{
  // HumanData(const int id, const double x, const double y)
  //   : id{id}, x{x}, y{y}, vx{0.0}, vy{0.0}, ang{0.0}
  // {}

  HumanData() : id{0}, x{0}, y{0}, vx{0}, vy{0}, ang{0}, ox{0}, oy{0}, oz{0}, ow{1}, confidence{0.01}
  {}

  HumanData(const int id, const double x, const double y, const double vx, const double vy, const double ang, const double ox, const double oy, const double oz, const double ow, const double confidence)
    : id{id}, x{x}, y{y}, vx{vx}, vy{vy}, ang{ang}
    , ox{ox}, oy{oy}, oz{oz}, ow{ow}, confidence{confidence}
  {}

  int id;
  double x, y, vx, vy, ang;
  double ox, oy, oz, ow;
  double confidence;
};

#endif /* STRUCT_DEFS_HPP */

