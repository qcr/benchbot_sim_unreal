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

  ISAAC_PARAM(std::string, odom_channel_name, "/odom");
  ISAAC_PARAM(std::string, ros_robot_frame, "base_link");

  ISAAC_PARAM(std::string, gt_robot_frame, "gt_robot");
  ISAAC_PARAM(std::string, gt_world_frame, "gt_world");

  ISAAC_PARAM(std::string, ros_odom_frame, "odom");
  ISAAC_PARAM(std::string, ros_world_frame, "map");

 private:
  struct RosData;
  std::unique_ptr<RosData> ros_data_;
};

}  // namespace benchbot

ISAAC_ALICE_REGISTER_CODELET(benchbot::OdomPublisher);
