#include "ground_truth_declarer.hh"

#include "messages/math.hpp"

namespace benchbot {

void GroundTruthDeclarer::start() { tickOnMessage(rx_unreal_poses()); }

void GroundTruthDeclarer::stop() {}

void GroundTruthDeclarer::tick() {
  // Bail if we can't find the requested robot
  auto proto_poses = rx_unreal_poses().getProto();
  int robot_idx = -1;
  for (unsigned int i = 0; i < proto_poses.getNames().size(); i++) {
    if (std::strcmp(get_robot_name().c_str(),
                    proto_poses.getNames()[i].cStr()) == 0) {
      robot_idx = i;
      break;
    }
  }
  if (robot_idx < 0) {
    LOG_WARNING("Could not find rigidbody for robot with name: '%s'",
                get_robot_name().c_str());
    return;
  }

  // Declare the Ground Truth pose for the Isaac System
  node()->pose().set(
      get_gt_world_frame(), get_gt_robot_frame(),
      isaac::FromProto(proto_poses.getBodies()[robot_idx].getRefTBody()),
      getTickTime());
}

}  // namespace benchbot
