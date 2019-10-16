#pragma once

#include <memory>

#include "engine/alice/alice.hpp"

#include "std_srvs/Empty.h"

namespace benchbot {

class LocalisationPublisher : public isaac::alice::Codelet {
 public:
  LocalisationPublisher() {}
  virtual ~LocalisationPublisher() {}

  void start() override;
  void stop() override;
  void tick() override;

  ISAAC_PARAM(bool, ground_truth_mode, false);

  ISAAC_PARAM(std::string, noisy_robot_frame, "robot");
  ISAAC_PARAM(std::string, noisy_odom_frame, "odom");

  ISAAC_PARAM(std::string, gt_init_frame, "gt_robot_init");
  ISAAC_PARAM(std::string, gt_robot_frame, "gt_robot");
  ISAAC_PARAM(std::string, gt_world_frame, "gt_world");

  ISAAC_PARAM(std::string, tf_map_frame, "map");
  ISAAC_PARAM(std::string, tf_odom_frame, "odom");

  ISAAC_PARAM(std::string, initial_pose_channel_name, "/initialpose");

 private:
  bool callbackResetInitialPose(std_srvs::Empty::Request& req,
                                std_srvs::Empty::Response& resp);
  std::optional<isaac::Pose3d> world_to_init_;

  struct RosData;
  std::unique_ptr<RosData> ros_data_;
};

}  // namespace benchbot

ISAAC_ALICE_REGISTER_CODELET(benchbot::LocalisationPublisher);
