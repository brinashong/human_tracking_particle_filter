#include "human_tracking_particle_filter/grid_map_interface.hpp"

GridMapInterface::GridMapInterface(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
  // Load ROS params
  pnh.param<double>("map_resolution", map_resolution_, 0.05);
  pnh.param<double>("map_size_x", map_size_x_, 30.0);
  pnh.param<double>("map_size_y", map_size_y_, 25.0);
  pnh.param<double>("map_origin_x", map_origin_x_, 15.0);
  pnh.param<double>("map_origin_y", map_origin_y_, 12.5);
  pnh.param<std::string>("map_frame_id", map_frame_id_, "map");

  // Create subs and pubs
  grid_map_pub_ = pnh.advertise<grid_map_msgs::GridMap>("grid_map", 5, true);

  // Init grid map
  map_ptr_ = std::make_shared<grid_map::GridMap>();
  map_ptr_->setGeometry(grid_map::Length{map_size_x_, map_size_y_}, map_resolution_, grid_map::Position{map_origin_x_, map_origin_y_});
  map_ptr_->setFrameId(map_frame_id_);
  // Add probability layer, value range from 0.0 - 1.0
  map_ptr_->add({"probability_layer"}, 0.0);
  // Add obstacle layer, value range from 0.0 - 100.0
  // This layer will be translated to the local costmap
  map_ptr_->add({"obstacle_layer"}, 0.0);
}

GridMapInterface::~GridMapInterface()
{
}

void GridMapInterface::insertSemanticData(pedsim_msgs::SemanticData ha)
{
  // Insert semantic data to map
  for (auto const pt : ha.points)
  {
    //
  }
}

std::shared_ptr<grid_map::GridMap> GridMapInterface::getGridMap()
{
  if (map_ptr_)
  {
    return map_ptr_;
  }
  else
  {
    return nullptr;
  }
}
