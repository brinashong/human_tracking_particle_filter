#include "human_tracking_particle_filter/costmap_2d_plugin.hpp"
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(costmap_plugin::HumanTrackingParticleFilterLayer, costmap_2d::Layer)

namespace costmap_plugin
{
  HumanTrackingParticleFilterLayer::HumanTrackingParticleFilterLayer()
    : tf_listerner_{tf_buffer_}
  {
  }

  HumanTrackingParticleFilterLayer::~HumanTrackingParticleFilterLayer()
  {
  }

  void HumanTrackingParticleFilterLayer::onInitialize()
  {
    // Instantiate nodehandles for this costmap plugin
    pnh_ = ros::NodeHandle("~/" + name_);
    nh_ = ros::NodeHandle("");
    current_ = true;

    htpf_ptr_ = std::make_unique<HumanTrackingParticleFilter>(nh_, pnh_);
  }

  void HumanTrackingParticleFilterLayer::updateBounds(double robot_x,
      double robot_y,
      double robot_yaw,
      double *min_x,
      double *min_y,
      double *max_x,
      double *max_y)
  {
    ROS_INFO_STREAM("Human Tracking Particle Filter Layer Update Bounds!");
  }

  void HumanTrackingParticleFilterLayerupdateCosts(costmap_2d::Costmap2D &master_grid,
      int min_i,
      int min_j,
      int max_i,
      int max_j)
  {
    ROS_INFO_STREAM("Human Tracking Particle Filter Layer Update Costs!");
  }
} // costmap_plugin
