#ifndef HUMAN_TRACKING_PARTICLE_FILTER
#define HUMAN_TRACKING_PARTICLE_FILTER

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

#include "pedsim_msgs/SemanticData.h"

class HumanTrackingParticleFilter
{
  public:
    HumanTrackingParticleFilter(ros::NodeHandle &nh, ros::NodeHandle &pnh);
    ~HumanTrackingParticleFilter() {}

    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &input);
    void humanPositionCallback(const pedsim_msgs::SemanticData::ConstPtr &input);

  private:
    // ROS subscribers
    ros::Subscriber scan_sub_;
    ros::Subscriber human_sub_;

    // params
    double frequency_;
    std::string scan_topic_;
    std::string human_topic_;
};

#endif /* HUMAN_TRACKING_PARTICLE_FILTER */
