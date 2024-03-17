#include <memory>
#include "human_tracking_particle_filter/human_tracking_particle_filter.hpp"

HumanTrackingParticleFilter::HumanTrackingParticleFilter(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
  pnh.param<double>("frequency", frequency_, 1.0);
  pnh.param<std::string>("scan_topic", scan_topic_, "/scan");
  pnh.param<std::string>("human_topic", human_topic_, "/human");

  scan_sub_ = nh.subscribe<sensor_msgs::LaserScan>(scan_topic_, 1, [this](const sensor_msgs::LaserScan::ConstPtr& input){ this->laserScanCallback(input); });
  human_sub_ = nh.subscribe<pedsim_msgs::SemanticData>(human_topic_, 1, [this](const pedsim_msgs::SemanticData::ConstPtr& input){ this->humanPositionCallback(input); });
}

void HumanTrackingParticleFilter::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &input)
{
  ROS_INFO("Scan received!");
}

void HumanTrackingParticleFilter::humanPositionCallback(const pedsim_msgs::SemanticData::ConstPtr &input)
{
  ROS_INFO("Human position received!");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "human_tracking_particle_filter");
  ros::NodeHandle nh, pnh("~");

  std::shared_ptr<HumanTrackingParticleFilter> obj = std::make_shared<HumanTrackingParticleFilter>(nh, pnh);

  ros::spin();

  return 0;
}
