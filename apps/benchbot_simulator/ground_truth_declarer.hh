#pragma once

#include "engine/alice/alice.hpp"
#include "messages/bodies.capnp.h"

namespace benchbot {

class GroundTruthDeclarer : public isaac::alice::Codelet {
 public:
  GroundTruthDeclarer() {}
  virtual ~GroundTruthDeclarer() {}

  void start() override;
  void stop() override;
  void tick() override;

  ISAAC_PARAM(std::string, robot_name, "carter_1");

  ISAAC_PARAM(std::string, gt_robot_frame, "gt_robot");
  ISAAC_PARAM(std::string, gt_world_frame, "gt_world");

  ISAAC_PROTO_RX(RigidBody3GroupProto, unreal_poses);
};

}  // namespace benchbot

ISAAC_ALICE_REGISTER_CODELET(benchbot::GroundTruthDeclarer);
