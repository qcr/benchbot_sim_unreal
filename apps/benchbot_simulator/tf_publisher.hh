#pragma once

#include <memory>

#include "engine/alice/alice.hpp"

namespace benchbot {

class TfPublisher : public isaac::alice::Codelet {
 public:
  TfPublisher() {}
  virtual ~TfPublisher() {}

  void start() override;
  void stop() override;
  void tick() override;

  ISAAC_PARAM(std::string, isaac_base_frame, "base_link");
  ISAAC_PARAM(std::string, ros_base_frame, "");

  ISAAC_PARAM(std::vector<std::string>, isaac_child_frames, {});
  ISAAC_PARAM(std::vector<std::string>, ros_child_frames, {});

 private:
  struct RosData;
  std::unique_ptr<RosData> ros_data_;
};

}  // namespace benchbot

ISAAC_ALICE_REGISTER_CODELET(benchbot::TfPublisher);
