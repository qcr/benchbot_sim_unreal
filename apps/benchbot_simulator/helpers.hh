#pragma once

#include "engine/alice/components/Pose.hpp"
#include "geometry_msgs/Pose.h"
#include "tf/transform_broadcaster.h"

namespace benchbot {

inline geometry_msgs::Pose pose3d_to_pose(const isaac::Pose3d &pose) {
  geometry_msgs::Pose p;
  p.orientation.w = pose.rotation.quaternion().w();
  p.orientation.x = pose.rotation.quaternion().x();
  p.orientation.y = pose.rotation.quaternion().y();
  p.orientation.z = pose.rotation.quaternion().z();

  p.position.x = pose.translation.x();
  p.position.y = pose.translation.y();
  p.position.z = pose.translation.z();

  return p;
}

inline tf::Transform pose3d_to_transform(const isaac::Pose3d &pose) {
  return tf::Transform(tf::Quaternion(pose.rotation.quaternion().w(),
                                      pose.rotation.quaternion().x(),
                                      pose.rotation.quaternion().y(),
                                      pose.rotation.quaternion().z()),
                       tf::Vector3(pose.translation.x(), pose.translation.y(),
                                   pose.translation.z()));
}

}  // namespace benchbot
