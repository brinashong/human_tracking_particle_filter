#ifndef HUMAN_TRACKING_PARTICLE_FILTER_GRID_MAP_INTERFACE_HPP
#define HUMAN_TRACKING_PARTICLE_FILTER_GRID_MAP_INTERFACE_HPP

#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <human_tracking_particle_filter/struct_defs.hpp>
#include <human_tracking_particle_filter/const_defs.hpp>
#include <angles/angles.h>

#include <memory>

class GridMapInterface
{
public:
  GridMapInterface(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  ~GridMapInterface();

  /**
   * \brief Insert human data into grid map
   * \attention This function assumes the human poses are in the same frame as grid map
   */
  void insertHumanData(std::unordered_map<int, HumanData> hd, const double patch = 0.0);
  void createGridMap(const std::string &frame_id, const double map_size_x, const double map_size_y, const double map_resolution, const double map_origin_x = 0.0, const double map_origin_y = 0.0);
  void resetAllGridMapData();
  void resetObstacleGridMapData();
  void resetProbabilityGridMapData();
  std::shared_ptr<grid_map::GridMap> getGridMap();
  void publishGridMap();
  /**
     * @brief Get the radius object for update bounds
     *
     * @param cutoff cuttoff value
     * @param amplitude amplitude of gaussian distribution
     * @param covar_x x direction covariance
     * @param covar_y y direction covariance
     * @return double
     */
    static inline double const getRadius(const double cutoff, const double amplitude, const double covar_x, const double covar_y)
    {
      return (covar_x >= covar_y) ?
        std::sqrt(-2 * covar_x * std::log(cutoff / amplitude)) : std::sqrt(-2 * covar_y * std::log(cutoff / amplitude));
    }

    /**
     * @brief Get cost value from gaussian distribution
     *
     * @param x x position in gaussian distribution
     * @param y y position in gaussian distribution
     * @param x0 gaussian center point x
     * @param y0 gaussian center point y
     * @param amplitude amplitude of gaussian distribution
     * @param covar_x x direction covariance
     * @param covar_y y direction covariance
     * @return double cost value of gaussian
     *
     * @attention (removed) the result of cost value will be capped at 254
     *            due to how costmap_2d works
     *
     * @ref  https://en.wikipedia.org/wiki/Gaussian_function
     */
    static inline double const gaussian(const double x, const double y, const double x0, const double y0, const int amplitude, const double covar_x, const double covar_y, const double skew)
    {
      double dx = x - x0, dy = y - y0;
      double h = std::sqrt(dx * dx + dy * dy);
      double angle = std::atan2(dy, dx);
      double mx = std::cos(angle - skew) * h;
      double my = std::sin(angle - skew) * h;
      double f1 = std::pow(mx, 2.0) / (2.0 * covar_x),
             f2 = std::pow(my, 2.0) / (2.0 * covar_y);
      return amplitude * exp(-(f1 + f2));
    }

private:
  // ROS publisher
  ros::Publisher grid_map_pub_;

  // params
  std::shared_ptr<grid_map::GridMap> map_ptr_;
  double map_resolution_, map_size_x_, map_size_y_, map_origin_x_, map_origin_y_;
  double buffer_;
  std::string map_frame_id_;
};

#endif /* HUMAN_TRACKING_PARTICLE_FILTER_GRID_MAP_INTERFACE_HPP */
