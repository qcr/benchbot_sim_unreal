#pragma once

#include <memory>
#include <string>

#include "engine/alice/alice.hpp"
#include "messages/messages.hpp"

#include "geometry_msgs/Twist.h"

namespace benchbot {

class TwistSubscriber : public isaac::alice::Codelet {
 public:
  TwistSubscriber() {}
  virtual ~TwistSubscriber() {}

  void start() override;
  void stop() override;
  void tick() override;

  ISAAC_PARAM(std::string, twist_channel_name, "/cmd_vel");

  ISAAC_PROTO_TX(StateProto, cmd_vel);

 private:
  void callbackTwist(const geometry_msgs::Twist &msg);

  struct RosData;
  std::unique_ptr<RosData> ros_data_;
};

}  // namespace benchbot

ISAAC_ALICE_REGISTER_CODELET(benchbot::TwistSubscriber);
