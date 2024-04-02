#ifndef HUMAN_TRACKING_PARTICLE_FILTER_COSTMAP_PLUGIN_HPP
#define HUMAN_TRACKING_PARTICLE_FILTER_COSTMAP_PLUGIN_HPP

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <human_tracking_particle_filter/HTPFLayerConfig.h>
#include <dynamic_reconfigure/server.h>
#include <memory>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/utils.h>

#include "human_tracking_particle_filter/grid_map_interface.hpp"

namespace costmap_plugin
{
  class HTPFLayer: public costmap_2d::Layer
  {
  public:
    // Constructor & destructor
    HTPFLayer();
    virtual ~HTPFLayer();

    // Three important functions
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
    std::shared_ptr<dynamic_reconfigure::Server<::costmap_plugin::HTPFLayerConfig>> dym_srv_;
    ros::NodeHandle pnh_;
    ros::NodeHandle nh_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listerner_;

    // std::unique_ptr<HumanTrackingParticleFilter> htpf_ptr_;
    std::unique_ptr<GridMapInterface> gm_ptr_;
    bool is_first_time_, debug_;
    // debug params
    double mark_x_, mark_y_;
    double gaussian_center_x_, gaussian_center_y_, amplitude_, covar_x_, covar_y_;
    double angle_, factor_, cutoff_, buffer_;
  };
} // namespace costmap_plugin

#endif /* HUMAN_TRACKING_PARTICLE_FILTER_COSTMAP_PLUGIN_HPP */
