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
  // Start a ROS node & delegate SIGINT handling to Isaac
  ros::M_string args;
  if (!ros::isInitialized()) {
    ros::init(args, "twist_subscriber", ros::init_options::NoSigintHandler);
  }

  // Initialise all of the ROS data we are going to need
  ros_data_ = std::make_unique<RosData>();
  ros_data_->nh.setCallbackQueue(&(ros_data_->callback_queue));
  ros_data_->sub = ros_data_->nh.subscribe(
      get_twist_channel_name(), 2, &TwistSubscriber::callbackTwist, this);

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
  // LOG_DEBUG("Received Twist from ROS; passing to Isaac SIM");

  // Form a Isaac message from the input twist message
  isaac::messages::DifferentialBaseControl imsg;
  imsg.linear_speed() = msg.linear.x;  // WTF is that magic syntax? Interesting
  imsg.angular_speed() = msg.angular.z;

  ToProto(imsg, tx_cmd_vel().initProto(), tx_cmd_vel().buffers());
  tx_cmd_vel().publish();
}

}  // namespace benchbot
