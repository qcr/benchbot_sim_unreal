#include "ros_pinger.hh"

#include "ros/callback_queue.h"
#include "ros/ros.h"
#include "std_msgs/String.h"

namespace isaac {

struct RosPinger::RosData {
  ros::NodeHandle nh;
  ros::Publisher pub;
  ros::CallbackQueue callback_queue;
};

void RosPinger::start() {
  ros::M_string args;
  if (!ros::isInitialized()) {
    ros::init(args, "ros_pinger", ros::init_options::NoSigintHandler);
  }

  ros_data_ = std::make_unique<RosData>();
  ros_data_->nh.setCallbackQueue(&(ros_data_->callback_queue));
  ros_data_->pub = ros_data_->nh.advertise<std_msgs::String>("/hello_ros", 1);

  tickPeriodically();
}

void RosPinger::stop() {
  ros_data_->pub.shutdown();
  ros_data_ = nullptr;
}

void RosPinger::tick() {
  LOG_WARNING("tick");
  if (ros::ok()) {
    std_msgs::String msg;
    msg.data = "Hello ROS, from Isaac!";
    ros_data_->pub.publish(msg);
  }
  ros_data_->callback_queue.callAvailable();
}

RosPinger::~RosPinger() {}
RosPinger::RosPinger() {}
}  // namespace isaac
