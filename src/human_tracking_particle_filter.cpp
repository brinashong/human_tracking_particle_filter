#include "human_tracking_particle_filter/human_tracking_particle_filter.hpp"
#include "human_tracking_particle_filter/particleFilterConfig.h"

HumanTrackingParticleFilter::HumanTrackingParticleFilter(ros::NodeHandle &nh, ros::NodeHandle &pnh)
{
  // Initialize dynamic reconfigure server
  dyn_srv_ = std::make_shared<dynamic_reconfigure::Server<human_tracking_particle_filter::particleFilterConfig>>();
  dyn_srv_->setCallback([this](human_tracking_particle_filter::particleFilterConfig& config, ...){this->reconfigureCB(config);});

  pnh.param<double>("frequency", frequency_, 1.0);
  pnh.param<std::string>("scan_topic", scan_topic_, "/scan");
  pnh.param<std::string>("human_topic", human_topic_, "/human");
  pnh.param<bool>("add_noise", add_noise_, false);
  pnh.param<double>("mean", mean_, 0.0);
  pnh.param<double>("std_dev", std_dev_, 1.0);

  scan_sub_ = nh.subscribe<sensor_msgs::LaserScan>(scan_topic_, 1, [this](const sensor_msgs::LaserScan::ConstPtr& input){ this->laserScanCallback(input); });
  human_sub_ = nh.subscribe<pedsim_msgs::SemanticData>(human_topic_, 1, [this](const pedsim_msgs::SemanticData::ConstPtr& input){ this->humanPositionCallback(input); });
}

void HumanTrackingParticleFilter::reconfigureCB(human_tracking_particle_filter::particleFilterConfig &config)
{
  this->add_noise_ = config.add_noise;
  this->frequency_ = config.frequency;
  this->mean_ = config.mean;
  this->std_dev_ = config.std_dev;
}

void HumanTrackingParticleFilter::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &input)
{
  ROS_INFO("Scan received!");
}

void HumanTrackingParticleFilter::humanPositionCallback(const pedsim_msgs::SemanticData::ConstPtr &input)
{
  ROS_INFO("Human position received!");

  // store the human positions
  humans_.clear();
  for (const auto &p : input->points)
  {
    humans_.emplace_back(p.location.x, p.location.y);
  }

  // apply Gaussian noise 
  if (add_noise_)
  {
    std::default_random_engine rand_gen;
    std::normal_distribution<double> gaussian(mean_, std_dev_);

    for (auto &p : humans_)
    {
      p.x += gaussian(rand_gen);
      p.y += gaussian(rand_gen);
    }
  }
}
