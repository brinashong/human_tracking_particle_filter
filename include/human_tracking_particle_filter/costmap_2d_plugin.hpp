#ifndef HUMAN_TRACKING_PARTICLE_FILTER_COSTMAP_2D_PLUGIN_HPP
#define HUMAN_TRACKING_PARTICLE_FILTER_COSTMAP_2D_PLUGIN_HPP

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

#include <memory>
#include "human_tracking_particle_filter/human_tracking_particle_filter.hpp"

namespace costmap_plugin
{
  class HumanTrackingParticleFilterLayer : public costmap_2d::Layer
  {
  public:
    HumanTrackingParticleFilterLayer();
    virtual ~HumanTrackingParticleFilterLayer();

    virtual void onInitialize() override;
    virtual void updateBounds(double robot_x,
                      double robot_y,
                      double robot_yaw,
                      double *min_x,
                      double *min_y,
                      double *max_x,
                      double *max_y) override;
    virtual void updateCosts(costmap_2d::Costmap2D &master_grid,
                     int min_i,
                     int min_j,
                     int max_i,
                     int max_j) override;
  private:
    ros::NodeHandle pnh_;
    ros::NodeHandle nh_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listerner_;

    std::unique_ptr<HumanTrackingParticleFilter> htpf_ptr_;
  };
} // costmap_plugin

#endif /* HUMAN_TRACKING_PARTICLE_FILTER_COSTMAP_2D_PLUGIN_HPP */
