#ifndef HUMAN_TRACKING_PARTICLE_FILTER_HPP
#define HUMAN_TRACKING_PARTICLE_FILTER_HPP

#include <vector>
#include <random>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <dynamic_reconfigure/server.h>

#include "human_tracking_particle_filter/struct_defs.hpp"
#include "human_tracking_particle_filter/particleFilterConfig.h"
#include "pedsim_msgs/SemanticData.h"

class HumanTrackingParticleFilter
{
  public:
    HumanTrackingParticleFilter(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    ~HumanTrackingParticleFilter() {}

    void reconfigureCB(human_tracking_particle_filter::particleFilterConfig &config);
    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &input);
    void humanPositionCallback(const pedsim_msgs::SemanticData::ConstPtr &input);

  private:
    // ROS subscribers
    ros::Subscriber scan_sub_;
    ros::Subscriber human_sub_;

    // dynamic reconfigure
    std::shared_ptr<dynamic_reconfigure::Server<human_tracking_particle_filter::particleFilterConfig>> dyn_srv_;

    // params
    double frequency_;
    std::string scan_topic_;
    std::string human_topic_;
    bool add_noise_;
    double mean_;
    double std_dev_;

    // variables
    std::vector<HumanData> humans_;
};

#endif /* HUMAN_TRACKING_PARTICLE_FILTER_HPP */
