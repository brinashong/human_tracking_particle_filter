#ifndef CONST_DEFS_HPP
#define CONST_DEFS_HPP

// GRID MAP LAYER NAME
inline constexpr const char* OBSTALE_LAYER {"obstacle_layer"};
inline constexpr const char* PROBABILITY_LAYER = {"probability_layer"};

// GRID MAP VALUE
inline constexpr double NO_INFORMATION = -1.0;
inline constexpr double LETHAL_OBSTACLE = 100.0;
inline constexpr double INSCRIBED_INFLATED_OBSTACLE = 99.0;
inline constexpr double FREE_SPACE = 0.0;

#endif /* CONST_DEFS_HPP */
