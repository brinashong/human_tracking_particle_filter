#ifndef STRUCT_DEFS_HPP
#define STRUCT_DEFS_HPP

struct HumanData
{
  HumanData(const double x, const double y)
    : x{x}, y{y}
  {}

  double x, y;
};

#endif /* STRUCT_DEFS_HPP */

