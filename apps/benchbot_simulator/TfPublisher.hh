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

  ISAAC_PARAM(std::string, tf_dynamic_channel_name, "/tf");
  ISAAC_PARAM(std::string, tf_static_channel_name, "/tf_static");

  ISAAC_PARAM(std::string, tf_base_frame);

  ISAAC_PARAM(std::vector<std::string>, tf_static_frames, {});
  ISAAC_PARAM(std::vector<std::string>, tf_dynamic_frames, {});

 private:
  struct RosData;
  std::unique_ptr<RosData> ros_data_;
};

}  // namespace benchbot

ISAAC_ALICE_REGISTER_CODELET(benchbot::TfPublisher);
