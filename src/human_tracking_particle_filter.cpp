#include "human_tracking_particle_filter/human_tracking_particle_filter.hpp"
#include "human_tracking_particle_filter/grid_map_interface.hpp"
#include "human_tracking_particle_filter/particleFilterConfig.h"

HumanTrackingParticleFilter::HumanTrackingParticleFilter(ros::NodeHandle &nh, ros::NodeHandle &pnh) 
  : tf2_listener_(tf2_buffer_)
{
  // Initialize dynamic reconfigure server
  dyn_srv_ = std::make_shared<dynamic_reconfigure::Server<human_tracking_particle_filter::particleFilterConfig>>();
  dyn_srv_->setCallback([this](human_tracking_particle_filter::particleFilterConfig& config, ...){this->reconfigureCB(config);});

  pnh.param<double>("frequency", frequency_, 1.0);
  pnh.param<std::string>("scan_topic", scan_topic_, "/scan");
  pnh.param<std::string>("human_topic", human_topic_, "/human");
  pnh.param<std::string>("human_frame", human_frame_, "map");
  pnh.param<std::string>("robot_frame", robot_frame_, "base_link");
  pnh.param<bool>("add_noise", add_noise_, false);
  pnh.param<double>("mean", mean_, 0.0);
  pnh.param<double>("std_dev", std_dev_, 1.0);

  pnh.param<int>("num_particles", num_particles_, 100);

  scan_sub_ = nh.subscribe<sensor_msgs::LaserScan>(scan_topic_, 1, [this](const sensor_msgs::LaserScan::ConstPtr& input){ this->laserScanCallback(input); });
  human_sub_ = nh.subscribe<pedsim_msgs::SemanticData>(human_topic_, 1, [this](const pedsim_msgs::SemanticData::ConstPtr& input){ this->humanPositionCallback(input); });

  grid_map_interface_ptr_ = std::make_unique<GridMapInterface>(nh, pnh);

  double init = 1.0 / num_particles_;
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
  ROS_INFO_THROTTLE(3.0, "Scan received!");
}

void HumanTrackingParticleFilter::humanPositionCallback(const pedsim_msgs::SemanticData::ConstPtr &input)
{
  ROS_INFO_THROTTLE(3.0, "Human position received!");

  // get transform from human to robot frame
  geometry_msgs::TransformStamped transform_stamped;
  try
  {
    transform_stamped = tf2_buffer_.lookupTransform(robot_frame_, human_frame_, ros::Time(0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN_STREAM(ex.what());
  }

  // store the human positions
  humans_.clear();
  geometry_msgs::Point new_p;
  for (const auto &p : input->points)
  {
    // apply transform
    tf2::doTransform(p.location, new_p, transform_stamped);
    humans_.emplace_back(new_p.x, new_p.y);
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

  grid_map_interface_ptr_->resetAllGridMapData();
  grid_map_interface_ptr_->insertHumanData(humans_);
}
