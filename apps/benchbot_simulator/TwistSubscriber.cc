#include "TwistSubscriber.hh"

#include "engine/gems/state/io.hpp"
#include "messages/state/differential_base.hpp"
#include "ros/callback_queue.h"
#include "ros/ros.h"

namespace benchbot {
struct TwistSubscriber::RosData {
  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::CallbackQueue callback_queue;
};

void TwistSubscriber::start() {
  ros::M_string args;
  if (!ros::isInitialized()) {
    ros::init(args, "twist_subscriber", ros::init_options::NoSigintHandler);
  }

  ros_data_ = std::make_unique<RosData>();
  ros_data_->nh.setCallbackQueue(&(ros_data_->callback_queue));
  ros_data_->sub = ros_data_->nh.subscribe(
      get_subscriber_channel_name(), 2, &TwistSubscriber::callbackTwist, this);

  tickPeriodically();
}

void TwistSubscriber::stop() {
  ros_data_->sub.shutdown();
  ros_data_ = nullptr;
}

void TwistSubscriber::tick() {
  if (ros::ok()) {
    ros_data_->callback_queue.callAvailable();
  } else {
    LOG_ERROR("Lost connection to ROS master; should shut down");
  }
}

void TwistSubscriber::callbackTwist(const geometry_msgs::Twist &msg) {
  LOG_WARNING("RECEIVED TWIST...");

  // Form a Isaac message from the input twist message
  // TODO had 2x linear velocity here... why?
  isaac::messages::DifferentialBaseControl imsg;
  imsg.linear_speed() = 2 * msg.linear.x;  // Why 2x? And WTF is that syntax???
  imsg.angular_speed() = 2 * msg.angular.x;

  ToProto(imsg, tx_base_cmd().initProto(), tx_base_cmd().buffers());
  tx_base_cmd().publish();
}

TwistSubscriber::~TwistSubscriber() {}
TwistSubscriber::TwistSubscriber() {}
}  // namespace benchbot
