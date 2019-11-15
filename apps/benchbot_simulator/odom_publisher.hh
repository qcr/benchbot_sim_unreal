#pragma once

#include <memory>

#include "engine/alice/alice.hpp"
#include "messages/messages.hpp"

namespace benchbot {

class OdomPublisher : public isaac::alice::Codelet {
 public:
  OdomPublisher() {}
  virtual ~OdomPublisher() {}

  void start() override;
  void stop() override;
  void tick() override;

  ISAAC_PROTO_RX(Odometry2Proto, robot_odom);

  ISAAC_PARAM(std::string, odom_channel_name, "/odom")
  ISAAC_PARAM(std::string, tf_base_frame, "base_link");

 private:
  struct RosData;
  std::unique_ptr<RosData> ros_data_;
};

}  // namespace benchbot

ISAAC_ALICE_REGISTER_CODELET(benchbot::OdomPublisher);
