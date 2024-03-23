#include <human_tracking_particle_filter/human_tracking_particle_filter.hpp>
#include <memory>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "human_tracking_particle_filter");
  ros::NodeHandle nh, pnh("~");

  std::shared_ptr<HumanTrackingParticleFilter> obj = std::make_shared<HumanTrackingParticleFilter>(nh, pnh);

  ros::spin();

  return 0;
}
