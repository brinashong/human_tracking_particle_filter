#include "human_tracking_particle_filter/costmap_plugin.hpp"
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(costmap_plugin::HTPFLayer, costmap_2d::Layer);

namespace costmap_plugin
{
  HTPFLayer::HTPFLayer()
    : tf_listerner_{tf_buffer_}
    , is_first_time_{true}
    , debug_{false}
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

    // htpf_ptr_ = std::make_unique<HumanTrackingParticleFilter>(nh_, pnh_);
    gm_ptr_ = std::make_unique<GridMapInterface>(nh_, pnh_);

    // Setup dynamic reconfigure server
    dym_srv_ = std::make_shared<dynamic_reconfigure::Server<::costmap_plugin::HTPFLayerConfig>> (pnh_);
    dym_srv_->setCallback(
        [this](::costmap_plugin::HTPFLayerConfig &config, ...)
        {
          enabled_ = config.enabled;
          gaussian_center_x_ = config.gaussian_center_x;
          gaussian_center_y_ = config.gaussian_center_y;
          amplitude_ = config.amplitude;
          covar_x_ = config.covar_x;
          covar_y_ = config.covar_y;
          angle_ = config.skew;
          factor_ = config.factor;
          cutoff_ = config.cutoff;
          buffer_ = config.buffer;
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
    if (!enabled_)
    {
      return;
    }

    if (is_first_time_)
    {
      // Create grid map based on local costmap
      // To enssure coverrage in local costmap
      auto sx = layered_costmap_->getCostmap()->getSizeInMetersX();
      auto sy = layered_costmap_->getCostmap()->getSizeInMetersY();
      auto map_size = std::sqrt(sx * sx + sy * sy);
      gm_ptr_->createGridMap(
          {"base_link"},
          map_size,
          map_size,
          layered_costmap_->getCostmap()->getResolution(),
          0.0,
          0.0
        );

      if (debug_)
      {
        ROS_INFO_STREAM(
            "\nFrame: " << "base_link"/* layered_costmap_->getCostmap(). */
            << "\n Size x: " << layered_costmap_->getCostmap()->getSizeInMetersX()
            << "\n Size y: " << layered_costmap_->getCostmap()->getSizeInMetersY()
            << "\n Resolution: " << layered_costmap_->getCostmap()->getResolution()
            << "\n Origin x: " << layered_costmap_->getCostmap()->getOriginX()
            << "\n Origin y: " << layered_costmap_->getCostmap()->getOriginY()
            );
      }
      is_first_time_ = false;
    }


    // Obtain current front position of robot
    mark_x_ = robot_x + std::cos(robot_yaw);
    mark_y_ = robot_y + std::sin(robot_yaw);

    // // Limit the update bounds to save computational waste
    // *min_x = std::min(*min_x, mark_x_);
    // *min_y = std::min(*min_y, mark_y_);
    // *max_x = std::max(*max_x, mark_x_);
    // *max_y = std::max(*max_y, mark_y_);
    ROS_INFO_STREAM_COND(debug_, "Human Tracking Particle Filter Layer Update Bounds!");
  }


  void HTPFLayer::updateCosts(
      costmap_2d::Costmap2D &master_grid,
      int min_i, 
      int min_j,
      int max_i,
      int max_j
      )
  {
    // Disable functionality if not enabled
    if (!enabled_)
    {
      return;
    }

    // Set master costmap_2d value
    // unsigned int mx, my;
    // if(master_grid.worldToMap(mark_x_, mark_y_, mx, my))
    // {
    //   master_grid.setCost(mx, my, costmap_2d::LETHAL_OBSTACLE);
    // }

    // // Example human
    // auto map_ptr = gm_ptr_->getGridMap();
    // double res = master_grid.getResolution();
    // double base = GridMapInterface::getRadius(cutoff_, amplitude_, covar_x_, covar_y_);
    // double point = buffer_ + GridMapInterface::getRadius(cutoff_, amplitude_, covar_x_ * factor_, covar_y_);
    // unsigned int width = std::max(1, static_cast<int>((base + point) / res));
    // unsigned int height = width;
    //
    // // Get gaussian center
    // double cx = gaussian_center_x_, cy = gaussian_center_y_;
    //
    // // Get submap index
    // double ox = cx + point / 2;
    // double oy = cy + point / 2;
    //
    // // Reset grid map before inputing new value
    // gm_ptr_->resetAllGridMapData();
    // if (grid_map::Index sm_idx; map_ptr->getIndex({ox, oy}, sm_idx))
    // {
    //   // Assuming it is a square
    //   auto sm_size = point / res;
    //   for (int i = 0; i < sm_size; ++i)
    //   {
    //     for (int j = 0; j < sm_size; ++j)
    //     {
    //       double x = ox - i * res, y = oy - j * res;
    //       double ma = atan2(y - cy, x - cx);
    //       double diff = angles::shortest_angular_distance(angle_, ma);
    //       if (fabs(diff) < M_PI / 2)
    //       {
    //         map_ptr->at(PROBABILITY_LAYER, {sm_idx.x() + i, sm_idx.y() + j}) = GridMapInterface::gaussian(
    //             ox - i * res, oy - j * res,
    //             cx, cy,
    //             amplitude_,
    //             factor_ * covar_x_, covar_y_,
    //             angle_
    //             );
    //       }
    //       else
    //       {
    //         map_ptr->at(PROBABILITY_LAYER, {sm_idx.x() + i, sm_idx.y() + j}) = GridMapInterface::gaussian(
    //             ox - i * res, oy - j * res,
    //             cx, cy,
    //             amplitude_,
    //             covar_x_, covar_y_,
    //             angle_
    //             );
    //       }
    //
    //     }
    //   }
    // }

    // gm_ptr_->insertHumanData({{gaussian_center_x_, gaussian_center_y_, factor_, 0.0, angle_, 0.0, 0.0, 0.0, 1.0}});
    gm_ptr_->publishGridMap();

    ROS_INFO_STREAM_COND(debug_, "Human Tracking Particle Filter Layer Update Costs!");
  }

} // namespace costmap_plugin
