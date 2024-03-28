#include "human_tracking_particle_filter/costmap_plugin.hpp"
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(costmap_plugin::HTPFLayer, costmap_2d::Layer);

namespace costmap_plugin
{
  HTPFLayer::HTPFLayer()
    : tf_listerner_{tf_buffer_}
  {
    ;
  }
  HTPFLayer::~HTPFLayer()
  {
    ;
  }


  void HTPFLayer::onInitialize()
  {
    // Instantiate nodehandles for this costmap plugin
    pnh_ = ros::NodeHandle("~/" + name_);
    nh_ = ros::NodeHandle("");
    current_ = true;

    htpf_ptr_ = std::make_unique<HumanTrackingParticleFilter>(nh_, pnh_);

    // Setup dynamic reconfigure server
    dym_srv_ = std::make_shared<dynamic_reconfigure::Server<::costmap_plugin::HTPFLayerConfig>> (nh_);
    dym_srv_->setCallback(
        [this](::costmap_plugin::HTPFLayerConfig &config, ...)
          {
            enabled_ = config.enabled;
          }
    );
  }


  void HTPFLayer::updateBounds(
      double robot_x,
      double robot_y,
      double robot_yaw,
      double * min_x,
      double * min_y,
      double * max_x,
      double * max_y
      )
  {
    // Disable functionality if not enabled
    if(!enabled_)
    {
      return;
    }

    // // Obtain current front position of robot
    // mark_x_ = robot_x + std::cos(robot_yaw);
    // mark_y_ = robot_y + std::sin(robot_yaw);

    // // Limit the update bounds to save computational waste
    // *min_x = std::min(*min_x, mark_x_);
    // *min_y = std::min(*min_y, mark_y_);
    // *max_x = std::max(*max_x, mark_x_);
    // *max_y = std::max(*max_y, mark_y_);
    ROS_INFO("Human Tracking Particle Filter Layer Update Bounds!");
  }


  void HTPFLayer::updateCosts(
      costmap_2d::Costmap2D & master_grid,
      int min_i, 
      int min_j,
      int max_i,
      int max_j
      )
  {
    // Disable functionality if not enabled
    if(!enabled_)
    {
      return;
    }

    // Set master costmap_2d value
    // unsigned int mx, my;
    // if(master_grid.worldToMap(mark_x_, mark_y_, mx, my))
    // {
    //   master_grid.setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);
    // }
    ROS_INFO("Human Tracking Particle Filter Layer Update Costs!");
  }
} // namespace costmap_plugin
