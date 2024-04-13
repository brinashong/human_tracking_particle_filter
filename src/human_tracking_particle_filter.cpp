#include "human_tracking_particle_filter/human_tracking_particle_filter.hpp"
#include "tf2/utils.h"
#include "visualization_msgs/MarkerArray.h"
#include <iomanip>

HumanTrackingParticleFilter::HumanTrackingParticleFilter(ros::NodeHandle &nh, ros::NodeHandle &pnh) 
  : tf2_listener_(tf2_buffer_)
{
  // Initialize dynamic reconfigure server
  dyn_srv_ = std::make_shared<dynamic_reconfigure::Server<human_tracking_particle_filter::particleFilterConfig>>();
  dyn_srv_->setCallback([this](human_tracking_particle_filter::particleFilterConfig& config, ...){this->reconfigureCB(config);});

  pnh.param<double>("frequency", frequency_, 1.0);
  pnh.param<std::string>("scan_topic", scan_topic_, "/scan");
  // pnh.param<std::string>("human_topic", human_topic_, "/human");
  pnh.param<std::string>("detection_topic", detection_topic_, "/detection");
  // pnh.param<std::string>("agent_state_topic", agent_state_topic_, "/agent_state");
  pnh.param<std::string>("human_frame", human_frame_, "map");
  pnh.param<std::string>("robot_frame", robot_frame_, "base_link");
  pnh.param<bool>("add_noise", add_noise_, false);
  pnh.param<double>("mean", mean_, 0.0);
  pnh.param<double>("std_dev", std_dev_, 1.0);
  pnh.param<double>("factor", factor_, 20.0);

  pnh.param<int>("num_particles", num_particles_, 10);

  main_timer_ = nh.createTimer(ros::Duration(1 / frequency_), &HumanTrackingParticleFilter::mainTimerCallback, this);

  scan_sub_ = nh.subscribe<sensor_msgs::LaserScan>(scan_topic_, 1, [this](const sensor_msgs::LaserScan::ConstPtr& input){ this->laserScanCallback(input); });
  // human_sub_ = nh.subscribe<pedsim_msgs::SemanticData>(human_topic_, 1, [this](const pedsim_msgs::SemanticData::ConstPtr& input){ this->humanPositionCallback(input); });
  detection_sub_ = nh.subscribe<object_detector_ros_messages::ObjectDetectMsg>(detection_topic_, 1, [this](const object_detector_ros_messages::ObjectDetectMsg::ConstPtr& input){ this->detectionCallback(input); });
  // agent_state_sub_ = nh.subscribe<pedsim_msgs::AgentStates>(agent_state_topic_, 1, [this](const pedsim_msgs::AgentStates::ConstPtr& input){ this->agentStatesCallback(input); });

  human_markers_pub_ = pnh.advertise<visualization_msgs::MarkerArray>("human_arrow", 1);
  particle_markers_pub_ = pnh.advertise<visualization_msgs::MarkerArray>("particles", 1);
  grid_map_interface_ptr_ = std::make_unique<GridMapInterface>(nh, pnh);

  std::string path = ros::package::getPath("human_tracking_particle_filter");
  ground_xt_file_.open(path + "/plots/ground_xt.txt");
  ground_yt_file_.open(path + "/plots/ground_yt.txt");
  noisy_xt_file_.open(path + "/plots/noisy_xt.txt");
  noisy_yt_file_.open(path + "/plots/noisy_yt.txt");
  mean_xt_file_.open(path + "/plots/mean_xt.txt");
  mean_yt_file_.open(path + "/plots/mean_yt.txt");
}

void HumanTrackingParticleFilter::reconfigureCB(human_tracking_particle_filter::particleFilterConfig &config)
{
  this->debug_ = config.debug;
  this->add_noise_ = config.add_noise;
  this->frequency_ = config.frequency;
  this->mean_ = config.mean;
  this->std_dev_ = config.std_dev;
  this->factor_ = config.factor;
}

void HumanTrackingParticleFilter::laserScanCallback(const sensor_msgs::LaserScan::ConstPtr &input)
{
  ROS_INFO_THROTTLE(3.0, "Scan received!");
}

// void HumanTrackingParticleFilter::humanPositionCallback(const pedsim_msgs::SemanticData::ConstPtr &input)
// {
//   ROS_INFO_THROTTLE(3.0, "Human position received!");
//
//   // get transform from human to robot frame
//   geometry_msgs::TransformStamped transform_stamped;
//   try
//   {
//     transform_stamped = tf2_buffer_.lookupTransform(robot_frame_, human_frame_, ros::Time(0));
//   }
//   catch (tf2::TransformException &ex)
//   {
//     ROS_WARN_STREAM(ex.what());
//   }
//
//   // store the human positions
//   humans_.clear();
//   geometry_msgs::Point new_p;
//   for (const auto &p : input->points)
//   {
//     // apply transform
//     tf2::doTransform(p.location, new_p, transform_stamped);
//     humans_[p.id] = {new_p.x, new_p.y};
//   }
//
//   // apply Gaussian noise 
//   if (add_noise_)
//   {
//     std::default_random_engine rand_gen;
//     std::normal_distribution<double> gaussian(mean_, std_dev_);
//
//     for (auto &p : humans_)
//     {
//       p.x += gaussian(rand_gen);
//       p.y += gaussian(rand_gen);
//     }
//   }
//
//   predict();
//
//   // grid_map_interface_ptr_->resetAllGridMapData();
//   // grid_map_interface_ptr_->insertHumanData(humans_);
// }

void HumanTrackingParticleFilter::detectionCallback(const object_detector_ros_messages::ObjectDetectMsg::ConstPtr &input)
{
  ROS_INFO("Detection received!");

  latest_detection_ = *input;
  // if (grid_map_interface_ptr_)
  // {
  //   grid_map_interface_ptr_->resetAllGridMapData();
  //   grid_map_interface_ptr_->insertHumanData(latest_humans_);
  //   grid_map_interface_ptr_->publishGridMap();
  // }
}

// void HumanTrackingParticleFilter::agentStatesCallback(const pedsim_msgs::AgentStates::ConstPtr &input)
// {
//   ROS_INFO_THROTTLE(3.0, "Agent states received!");
//   humans_.clear();
//   // Convert to human data format
//   for (const auto &in : input->agent_states)
//   {
//     humans_[in.id] = {
//       in.id,
//       in.pose.position.x,
//       in.pose.position.y,
//       in.twist.linear.x,
//       in.twist.linear.y,
//       tf2::getYaw(in.pose.orientation),
//       in.pose.orientation.x,
//       in.pose.orientation.y,
//       in.pose.orientation.z,
//       in.pose.orientation.w
//     };
//   }
//
//   if (debug_)
//   {
//     publishHumanMarker();
//   }
//
//   if (grid_map_interface_ptr_)
//   {
//     grid_map_interface_ptr_->resetAllGridMapData();
//     grid_map_interface_ptr_->insertHumanData(humans_);
//     grid_map_interface_ptr_->publishGridMap();
//   }
// }

void HumanTrackingParticleFilter::publishHumanMarker()
{
  int count = 0;
  visualization_msgs::MarkerArray ma;
  for (const auto &[id, human] : humans_)
  {
    visualization_msgs::Marker m;
    m.header.frame_id = human_frame_;
    m.pose.position.x = human.x;
    m.pose.position.y = human.y;
    m.pose.position.z = 0.5;

    m.pose.orientation.x = human.ox;
    m.pose.orientation.y = human.oy;
    m.pose.orientation.z = human.oz;
    m.pose.orientation.w = human.ow;

    m.color.r = 0.0f;
    m.color.g = 0.0f;
    m.color.b = 1.0f;
    m.color.a = 1.0;

    m.scale.x = 0.5;      // arrow length
    m.scale.y = 0.1;      // arrow width
    m.scale.z = 0.1;      // arrow height
    m.type = visualization_msgs::Marker::ARROW;
    m.action = visualization_msgs::Marker::ADD;

    // Any marker sent with the same namespace and id will overwrite the old one
    m.ns = "human_orientation";
    m.id = ++count;

    m.lifetime = ros::Duration(10.0);

    ma.markers.push_back(m);
  }

  human_markers_pub_.publish(ma);
}

void HumanTrackingParticleFilter::publishParticleMarker()
{
  int count = 0;
  visualization_msgs::MarkerArray ma;
  for (const auto &[id, particles] : particles_)
  {
    for (const auto &p : particles)
    {
      visualization_msgs::Marker m;
      m.header.frame_id = human_frame_;
      m.pose.position.x = p.x;
      m.pose.position.y = p.y;
      m.pose.position.z = 0.1;

      m.pose.orientation.x = humans_[id].ox;
      m.pose.orientation.y = humans_[id].oy;
      m.pose.orientation.z = humans_[id].oz;
      m.pose.orientation.w = humans_[id].ow;

      m.color.r = 1.0f;
      m.color.g = 0.0f;
      m.color.b = 0.0f;
      m.color.a = 1.0;

      m.scale.x = p.w + 0.2;      // arrow length
      m.scale.y = 0.05;      // arrow width
      m.scale.z = 0.05;      // arrow height
      m.type = visualization_msgs::Marker::ARROW;
      m.action = visualization_msgs::Marker::ADD;

      // Any marker sent with the same namespace and id will overwrite the old one
      m.ns = "particles";
      m.id = ++count;

      m.lifetime = ros::Duration(1.0);

      ma.markers.push_back(m);
    }
  }

  particle_markers_pub_.publish(ma);
}

void HumanTrackingParticleFilter::mainTimerCallback(const ros::TimerEvent& event)
{
  time_ = ros::Time::now();

  predict();
  measure();
  resample();

  std::unordered_map<int, HumanData> weighted_means = humans_;

  if (grid_map_interface_ptr_)
  {
    grid_map_interface_ptr_->resetAllGridMapData();

    // compute weighted mean of particle positions
    Particle total;
    for (const auto& [id, particles] : particles_)
    {
      total.x = 0.0;
      total.y = 0.0;
      total.w = 0.0;

      for (const auto &p : particles)
      {
        total.x += p.x * p.w;
        total.y += p.y * p.w;
        total.w += p.w;
      }

      weighted_means[id].x = total.x / total.w;
      weighted_means[id].y = total.y / total.w;

      // store mean data
      if (mean_xt_file_.is_open())
      {
        mean_xt_file_ << time_ << " " << std::fixed << std::setprecision(3) << weighted_means[id].x << std::endl;
        mean_yt_file_ << time_ << " " << std::fixed << std::setprecision(3) << weighted_means[id].y << std::endl;
      }
    }

    grid_map_interface_ptr_->insertHumanData(weighted_means);

    grid_map_interface_ptr_->publishGridMap();
  }

}

void HumanTrackingParticleFilter::predict()
{
  ROS_WARN("Predicting particles...");
  for (const auto &[id, h] : humans_)
  {
    // create a tracker for new detection
    if (particles_.find(id) == particles_.end())
    {
      initializeTracker(h);
    }

    std::random_device rd;
    std::mt19937 rand_gen(rd());
    std::normal_distribution<double> gaussian(mean_, std_dev_);
    for (int index = 0; index < num_particles_; index++)
    {
      // propagating the particles using constant velocity model, adding noise
      // particles_[h.id][index].x += (h.vx + gaussian(rand_gen)) / frequency_;
      // particles_[h.id][index].y += (h.vy + gaussian(rand_gen)) / frequency_;
      particles_[h.id][index].x += h.vx / frequency_;
      particles_[h.id][index].y += h.vy / frequency_;
    }
  }

  for (const auto &[id, particles] : particles_)
  {
    ROS_INFO_STREAM("Tracker for human " << id << " is active!");
    for (const auto &p : particles)
    {
      std::cout << "px: " << p.x << ", py: " << p.y << ", pw: " << p.w << std::endl;
    }
  }
}

void HumanTrackingParticleFilter::measure()
{
  ROS_WARN("Measuring latest human positions...");

  humans_.clear();

  // std::default_random_engine rand_gen;
  std::random_device rd;
  std::mt19937 rand_gen(rd());
  std::normal_distribution<double> gaussian(mean_, std_dev_);

  // Convert to human data format
  for (const auto &in : latest_detection_.ObjArray)
  {
    // rotate by -90deg
    tf2::Quaternion q_orig, q_rot, q_new;
    tf2::convert(in.ObjectPose.orientation , q_orig);
    q_rot.setRPY(0.0, 0.0, -M_PI_2);
    q_new = q_rot * q_orig;
    q_new.normalize();
    geometry_msgs::Quaternion msg = tf2::toMsg(q_new);

    double yaw = tf2::getYaw(msg);
    double vx = in.ObjectVel.linear.x * std::cos(yaw);
    double vy = in.ObjectVel.linear.x * std::sin(yaw);
    double x = in.ObjectPose.position.x;
    double y = in.ObjectPose.position.y;

    // store ground truth
    if (ground_xt_file_.is_open())
    {
      ground_xt_file_ << time_ << " " << std::fixed << std::setprecision(3) << x << std::endl;
      ground_yt_file_ << time_ << " " << std::fixed << std::setprecision(3) << y << std::endl;
    }

    // apply Gaussian noise 
    if (add_noise_)
    {
      x += gaussian(rand_gen);
      y += gaussian(rand_gen);
    }

    // store noisy data
    if (noisy_xt_file_.is_open())
    {
      noisy_xt_file_ << time_ << " " << std::fixed << std::setprecision(3) << x << std::endl;
      noisy_yt_file_ << time_ << " " << std::fixed << std::setprecision(3) << y << std::endl;
    }

    humans_[in.object_idx] = {
      in.object_idx,
      x,
      y,
      vx,
      vy,
      yaw,
      msg.x,
      msg.y,
      msg.z,
      msg.w
    };
  }

  if (debug_)
  {
    publishHumanMarker();
  }

  // loop through each tracker
  double sum_w = 0.0;
  std::unordered_set<int> id_to_erase;
  for (auto &[id, particles] : particles_)
  {
    sum_w = 0.0;
    // deal with missing detections
    if (humans_.find(id) == humans_.end())
    {
      if (missing_count_[id]++ > 3)
      {
        id_to_erase.insert(id);
      }
    }
    else
    {
      missing_count_[id] = 0;
      for (auto &p : particles)
      {
        // dist from the associated detection
        double dist = std::hypot((humans_[id].x - p.x), (humans_[id].y - p.y));

        p.w *= std::exp(-dist);
        // p.w *= std::exp(-std::pow(dist, 2) / (2.0 * std::pow(std_dev_, 2)));

        sum_w += p.w;
      }

      // normalize weights
      for (auto &p : particles)
      {
        p.w /= sum_w;
      }
    }
  }

  for (const auto &id : id_to_erase)
  {
    ROS_WARN_STREAM("Stopping tracker for human " << id);
    particles_.erase(id);
    missing_count_.erase(id);
  }

  publishParticleMarker();
}

void HumanTrackingParticleFilter::resample()
{
  for (auto &[id, particles] : particles_)
  {
    std::vector<Particle> new_particles(num_particles_);

    // Compute step size
    double step = 1.0 / num_particles_;

    // Generate a random starting point for systematic resampling
    std::random_device rd;
    std::mt19937 rand_gen(rd());
    std::uniform_real_distribution<double> uniform(0.0, 1.0);
    double cumulative_weight = uniform(rand_gen) * step;
    int index = 0;

    // Resample particles
    for (int i = 0; i < num_particles_; ++i)
    {
      double target_weight = i * step;
      while (cumulative_weight < target_weight)
      {
        cumulative_weight += particles[index].w;
        index++;
      }
      new_particles[i] = particles[index];
    }

    // Replace the old particles with the new set of particles
    particles = std::move(new_particles);
  }
}

void HumanTrackingParticleFilter::initializeTracker(const HumanData &h)
{
  ROS_WARN_STREAM("Initializing tracker for human " << h.id);

  // uniform distribution of particles within some radius around detection
  particles_[h.id] = {};
  Particle p;
  std::random_device rd;
  std::mt19937 rand_gen(rd());
  // std::uniform_real_distribution<double> uniform(-1.0, 1.0);
  std::normal_distribution<double> gaussian(mean_, std_dev_);

  for(int i = 0; i < num_particles_; i++)
  {
    // p.x = h.x + uniform(rand_gen);
    // p.y = h.y + uniform(rand_gen);
    p.x = h.x + gaussian(rand_gen);
    p.y = h.y + gaussian(rand_gen);
    p.w = 1.0 / num_particles_;
    particles_[h.id].push_back(p);
    
    std::cout << "ori x: " << h.x << ", y: " << h.y << ", new x: " << p.x << ", new y: " << p.y << std::endl;
  }

  missing_count_[h.id] = 0;
}
