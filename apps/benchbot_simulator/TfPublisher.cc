#include "TfPublisher.hh"

#include "ros/callback_queue.h"
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"

namespace benchbot {

struct TfPublisher::RosData {
  ros::NodeHandle nh;
  tf::TransformBroadcaster tf_broadcaster;
};

void TfPublisher::start() {
  // Start a ROS node & delegate SIGINT handling to Isaac
  ros::M_string args;
  if (!ros::isInitialized()) {
    ros::init(args, "tf_static_publisher", ros::init_options::NoSigintHandler);
  }
  //
  // Initialise all of the ROS data we are going to need
  ros_data_ = std::make_unique<RosData>();

  // Configure the codelet to only tick when we receive a new camera message
  // from the simulator
  tickPeriodically();
}

void TfPublisher::stop() { ros_data_ = nullptr; }

tf::Transform _pose3d_to_transform(const isaac::Pose3d &pose) {
  return tf::Transform(tf::Quaternion(pose.rotation.quaternion().w(),
                                      pose.rotation.quaternion().x(),
                                      pose.rotation.quaternion().y(),
                                      pose.rotation.quaternion().z()),
                       tf::Vector3(pose.translation.x(), pose.translation.y(),
                                   pose.translation.z()));
}

void TfPublisher::tick() {
  if (ros::ok()) {
    // LOG_DEBUG("Publishing tfs");

    // Setup information needed for our transforms
    std::optional<isaac::Pose3d> robot_to_frame;
    ros::Time ros_time = ros::Time::now();

    // Get all of the static tfs that have not already been published
    for (const std::string &s : get_tf_static_frames()) {
      // LOG_INFO("Getting static tf for: %s", s.c_str());
      robot_to_frame =
          node()->pose().tryGet(get_tf_base_frame(), s, getTickTime());
      if (robot_to_frame) {
        ros_data_->tf_broadcaster.sendTransform(
            tf::StampedTransform(_pose3d_to_transform(*robot_to_frame),
                                 ros_time, get_tf_base_frame(), s));
      }
    }

    // Get all of the dynamic tfs
    for (const std::string &s : get_tf_dynamic_frames()) {
      // LOG_INFO("Getting dynamic tf for: %s", s.c_str());
      robot_to_frame =
          node()->pose().tryGet(get_tf_base_frame(), s, getTickTime());
      if (robot_to_frame) {
        ros_data_->tf_broadcaster.sendTransform(
            tf::StampedTransform(_pose3d_to_transform(*robot_to_frame),
                                 ros_time, get_tf_base_frame(), s));
      } else {
        // LOG_ERROR("Failed to find tf for: %s", s.c_str());
      }
    }
  } else {
    LOG_ERROR("Lost connection to ROS master; should shut down");
  }
}

}  // namespace benchbot
