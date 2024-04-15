#include "human_tracking_particle_filter/grid_map_interface.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "human_tracking_particle_filter/const_defs.hpp"

GridMapInterface::GridMapInterface(ros::NodeHandle &nh, ros::NodeHandle &pnh)
  : buffer_{0.0}
{
  // Load ROS params
  pnh.param<double>("map_resolution", map_resolution_, 0.1);
  pnh.param<double>("map_size_x", map_size_x_, 30.0);
  pnh.param<double>("map_size_y", map_size_y_, 25.0);
  pnh.param<double>("map_origin_x", map_origin_x_, 15.0);
  pnh.param<double>("map_origin_y", map_origin_y_, 12.5);
  pnh.param<double>("obstacle_patch_buffer", buffer_, 0.0);
  pnh.param<std::string>("map_frame_id", map_frame_id_, "map");

  // Create subs and pubs
  grid_map_pub_ = pnh.advertise<grid_map_msgs::GridMap>("grid_map", 5, true);

  // Init grid map
  map_ptr_ = std::make_shared<grid_map::GridMap>();
  map_ptr_->setGeometry(grid_map::Length{map_size_x_, map_size_y_}, map_resolution_, grid_map::Position{map_origin_x_, map_origin_y_});
  map_ptr_->setFrameId(map_frame_id_);
  // Add probability layer, value range from 0.0 - 1.0
  map_ptr_->add({PROBABILITY_LAYER}, 0.0);
  // Add obstacle layer, value range from 0.0 - 100.0
  // This layer will be translated to the local costmap
  map_ptr_->add({OBSTACLE_LAYER}, 0.0);
}

GridMapInterface::~GridMapInterface()
{
}

void GridMapInterface::insertHumanData(std::unordered_map<int, HumanData> hd, const double patch)
{
  // Insert human data to map
  for (auto const &[id, pt] : hd)
  {
    if (grid_map::Index idx; map_ptr_->getIndex({pt.x, pt.y}, idx))
    {
      // Add human
      double base = GridMapInterface::getRadius(1.0, 100.0, 0.1, 0.1);
      double point = /* buffer_ */ patch + GridMapInterface::getRadius(1.0, 100.0, 0.1 * pt.vx, 0.1);
      unsigned int width = std::max(1, static_cast<int>((base + point) / map_resolution_));
      unsigned int height = width;

      // Get gaussian center
      double cx = pt.x, cy = pt.y;

      // Get submap index
      double ox = cx + point / 2;
      double oy = cy + point / 2;

      // Reset grid map before inputing new value
      if (grid_map::Index sm_idx; map_ptr_->getIndex({ox, oy}, sm_idx))
      {
        // Assuming it is a square
        auto sm_size = point / map_resolution_;
        for (int i = 0; i < sm_size; ++i)
        {
          for (int j = 0; j < sm_size; ++j)
          {
            double x = ox - i * map_resolution_, y = oy - j * map_resolution_;
            double ma = atan2(y - cy, x - cx);
            double diff = angles::shortest_angular_distance(pt.ang, ma);
            if (fabs(diff) < M_PI / 2)
            {
              map_ptr_->at(PROBABILITY_LAYER, {sm_idx.x() + i, sm_idx.y() + j}) = GridMapInterface::gaussian(
                  ox - i * map_resolution_, oy - j * map_resolution_,
                  cx, cy,
                  100.0,
                  (pt.vx > 0.0) ? pt.vx * 0.1 : 0.1 , 0.1,
                  pt.ang
                  );
            }
            else
            {
              map_ptr_->at(PROBABILITY_LAYER, {sm_idx.x() + i, sm_idx.y() + j}) = GridMapInterface::gaussian(
                  ox - i * map_resolution_, oy - j * map_resolution_,
                  cx, cy,
                  100.0,
                  0.1, 0.1,
                  pt.ang
                  );
            }

          }
        }
      }
      // map_ptr_->at(OBSTACLE_LAYER, idx) = LETHAL_OBSTACLE;
      // map_ptr_->at(PROBABILITY_LAYER, idx) = 1.0;
    }
  }
  publishGridMap();
}

void GridMapInterface::createGridMap(const std::string &frame_id, const double map_size_x, const double map_size_y, const double map_resolution, const double map_origin_x, const double map_origin_y)
{
  // Create an entirely new grid map
  map_ptr_ = std::make_shared<grid_map::GridMap>();
  // Update params
  map_frame_id_ = frame_id;
  map_size_x_ = map_size_x;
  map_size_y_ = map_size_y;
  map_resolution_ = map_resolution;
  map_origin_x_ = map_origin_x;
  map_origin_y_ = map_origin_y;

  map_ptr_->setGeometry(grid_map::Length{map_size_x_, map_size_y_}, map_resolution_, grid_map::Position{map_origin_x_, map_origin_y_});
  map_ptr_->setFrameId(map_frame_id_);
  // Add probability layer, value range from 0.0 - 1.0
  map_ptr_->add({PROBABILITY_LAYER}, 0.0);
  // Add obstacle layer, value range from 0.0 - 100.0
  // This layer will be translated to the local costmap
  map_ptr_->add({OBSTACLE_LAYER}, 0.0);
}

void GridMapInterface::resetAllGridMapData()
{
  resetObstacleGridMapData();
  resetProbabilityGridMapData();
}

void GridMapInterface::resetObstacleGridMapData()
{
  (*map_ptr_)[OBSTACLE_LAYER].setZero();
}

void GridMapInterface::resetProbabilityGridMapData()
{
  (*map_ptr_)[PROBABILITY_LAYER].setZero();
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

void GridMapInterface::publishGridMap()
{
  map_ptr_->setTimestamp(ros::Time::now().toNSec());
  // Create grid map msg
  grid_map_msgs::GridMap msg;
  grid_map::GridMapRosConverter::toMessage(*map_ptr_, msg);
  grid_map_pub_.publish(msg);
}
