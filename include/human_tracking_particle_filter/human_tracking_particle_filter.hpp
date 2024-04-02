#ifndef HUMAN_TRACKING_PARTICLE_FILTER_HPP
#define HUMAN_TRACKING_PARTICLE_FILTER_HPP

#include <algorithm>
#include <vector>
#include <random>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <dynamic_reconfigure/server.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include "human_tracking_particle_filter/grid_map_interface.hpp"
#include "human_tracking_particle_filter/struct_defs.hpp"
#include "human_tracking_particle_filter/particleFilterConfig.h"
#include <pedsim_msgs/SemanticData.h>
#include <pedsim_msgs/AgentStates.h>
#include <visualization_msgs/MarkerArray.h>

class HumanTrackingParticleFilter
{
  public:
    HumanTrackingParticleFilter(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    ~HumanTrackingParticleFilter() {}

    void reconfigureCB(human_tracking_particle_filter::particleFilterConfig &config);
    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &input);
    void humanPositionCallback(const pedsim_msgs::SemanticData::ConstPtr &input);
    void agentStatesCallback(const pedsim_msgs::AgentStates::ConstPtr &input);
    void publishHumanMarker();

  private:
    // bool update();
    // void prediction();
    // void measurement();
    // void resampling();

    // ROS subscribers
    ros::Subscriber scan_sub_;
    ros::Subscriber human_sub_;
    ros::Subscriber agent_state_sub_;
    ros::Publisher human_markers_pub_;

    // dynamic reconfigure
    std::shared_ptr<dynamic_reconfigure::Server<human_tracking_particle_filter::particleFilterConfig>> dyn_srv_;

    // params
    double frequency_;
    std::string scan_topic_;
    std::string human_topic_;
    std::string agent_state_topic_;
    std::string human_frame_;
    std::string robot_frame_;
    bool add_noise_;
    double mean_;
    double std_dev_;

    tf2_ros::Buffer tf2_buffer_;
    tf2_ros::TransformListener tf2_listener_;

    // grid map interface pointer
    std::unique_ptr<GridMapInterface> grid_map_interface_ptr_;

    // variables
    std::vector<HumanData> humans_;
    int num_particles_;
    double factor_;
    bool debug_;
};

#endif /* HUMAN_TRACKING_PARTICLE_FILTER_HPP */
