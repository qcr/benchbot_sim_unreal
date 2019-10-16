#pragma once

#include <memory>

#include "engine/alice/alice.hpp"
#include "messages/messages.hpp"

namespace benchbot {

class DepthPublisher : public isaac::alice::Codelet {
 public:
  DepthPublisher() {}
  virtual ~DepthPublisher() {}

  void start() override;
  void stop() override;
  void tick() override;

  ISAAC_PARAM(std::string, depth_channel_name, "/camera/depth/image_raw");
  ISAAC_PARAM(std::string, depth_frame_name, "robot_left_camera");

  ISAAC_PROTO_RX(DepthCameraProto, camera_depth);

 private:
  struct RosData;
  std::unique_ptr<RosData> ros_data_;
};

}  // namespace benchbot

ISAAC_ALICE_REGISTER_CODELET(benchbot::DepthPublisher);
