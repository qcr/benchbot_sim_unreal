#pragma once

#include "engine/alice/components/Pose.hpp"
#include "geometry_msgs/Pose.h"
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"

namespace benchbot {

constexpr double EPSILON = 1e-5;

inline bool equal(double x, double y) { return std::abs(x - y) <= EPSILON; }

inline isaac::Quaternion<double> safe_quaternion(const isaac::Pose3d &pose) {
  // For whatever reason I can have a Pose with a rotation of RPY = (0, 0, 0),
  // and the quaternion() method will give me a quaternion with wxyz values of
  // (0, 0, 0, 0)... aka rubbish. That's a real nasty bug...
  auto rpy = pose.rotation.eulerAnglesRPY();
  return (!equal(rpy.x(), 0.0) || !equal(rpy.y(), 0.0) || !equal(rpy.z(), 0.0))
             ? pose.rotation.quaternion()
             : isaac::Quaternion<double>(1.0, 0.0, 0.0, 0.0);
}

inline geometry_msgs::Pose pose3d_to_pose(const isaac::Pose3d &pose) {
  geometry_msgs::Pose p;
  isaac::Quaternion<double> q = safe_quaternion(pose);
  p.orientation.w = q.w();
  p.orientation.x = q.x();
  p.orientation.y = q.y();
  p.orientation.z = q.z();

  p.position.x = pose.translation.x();
  p.position.y = pose.translation.y();
  p.position.z = pose.translation.z();

  return p;
}

inline tf::Transform pose3d_to_transform(const isaac::Pose3d &pose) {
  isaac::Quaternion<double> q = safe_quaternion(pose);
  return tf::Transform(tf::Quaternion(q.x(), q.y(), q.z(), q.w()),
                       tf::Vector3(pose.translation.x(), pose.translation.y(),
                                   pose.translation.z()));
}

inline void start_ros_node() {
  if (!ros::isInitialized())
    ros::init(ros::M_string(), "benchbot_simulator",
              ros::init_options::NoSigintHandler);
}

}  // namespace benchbot
