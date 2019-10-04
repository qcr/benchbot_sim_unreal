#include "LidarPublisher.hh"

#include <algorithm>
#include <fstream>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

namespace benchbot {

struct LidarPublisher::RosData {
  ros::NodeHandle nh;
  ros::Publisher pub;
};

void LidarPublisher::start() {
  // Start a ROS node & delegate SIGINT handling to Isaac
  ros::M_string args;
  if (!ros::isInitialized()) {
    ros::init(args, "lidar_publisher", ros::init_options::NoSigintHandler);
  }

  // Initialise all of the ROS data we are going to need
  ros_data_ = std::make_unique<RosData>();
  ros_data_->pub = ros_data_->nh.advertise<sensor_msgs::LaserScan>(
      get_lidar_channel_name(), 2);

  // Configure the codelet to only tick when we receive a new camera message
  // from the simulator
  tickOnMessage(rx_lidar_scan());
}

void LidarPublisher::stop() {
  ros_data_->pub.shutdown();
  ros_data_ = nullptr;
}

void LidarPublisher::tick() {
  if (ros::ok()) {
    // LOG_DEBUG("Received RGB from Isaac; passing to ROS");

    // Received a message, cache time ASAP
    ros::Time msg_time = ros::Time::now();

    // Turn the Isaac message into a ROS message
    auto lidar_proto = rx_lidar_scan().getProto();
    std::ofstream o;
    o.open("/tmp/lidar/" + std::to_string(msg_time.toSec()) + ".txt");
    o << "Range Denormaliser: " << lidar_proto.getRangeDenormalizer()
      << std::endl;
    o << "Intensity Denormaliser: " << lidar_proto.getIntensityDenormalizer()
      << std::endl;
    o << "Delta Time: " << lidar_proto.getDeltaTime() << std::endl;
    o << "Invalid Threshold: " << lidar_proto.getInvalidRangeThreshold()
      << std::endl;
    o << "Out of Threshold: " << lidar_proto.getOutOfRangeThreshold()
      << std::endl;
    o << "Rays (range, intensity): " << std::endl;
    for (auto const &r : lidar_proto.getRays())
      o << '\t' << r.getRange() << ',' << int(r.getIntensity()) << std::endl;
    o << "Thetas: " << std::endl;
    for (auto const &t : lidar_proto.getTheta()) o << '\t' << t << std::endl;
    o << "Phis: " << std::endl;
    for (auto const &p : lidar_proto.getPhi()) o << '\t' << p << std::endl;
    o.close();

    // TODO header
  } else {
    LOG_ERROR("Lost connection to ROS master; should shut down");
  }
}

}  // namespace benchbot
