#ifndef HUMAN_TRACKING_PARTICLE_FILTER_GRID_MAP_INTERFACE_HPP
#define HUMAN_TRACKING_PARTICLE_FILTER_GRID_MAP_INTERFACE_HPP

#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <human_tracking_particle_filter/struct_defs.hpp>
#include <human_tracking_particle_filter/const_defs.hpp>

#include <memory>

class GridMapInterface
{
public:
  GridMapInterface(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  ~GridMapInterface();

  /**
   * \brief Insert human data into grip map
   * \attention This function assumes the human pose are in the same frame as grid map
   */
  void insertHumanData(std::vector<HumanData> hd);
  void resetAllGridMapData();
  void resetObstacleGridMapData();
  void resetProbabilityGridMapData();
  std::shared_ptr<grid_map::GridMap> getGridMap();
  void publishGridMap();

private:
  // ROS publisher
  ros::Publisher grid_map_pub_;

  // params
  std::shared_ptr<grid_map::GridMap> map_ptr_;
  double map_resolution_, map_size_x_, map_size_y_, map_origin_x_, map_origin_y_;
  std::string map_frame_id_;
};

#endif /* HUMAN_TRACKING_PARTICLE_FILTER_GRID_MAP_INTERFACE_HPP */
