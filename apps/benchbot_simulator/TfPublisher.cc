#include "TfPublisher.hh"
#include "helpers.hh"

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

  // Initialise all of the ROS data we are going to need
  ros_data_ = std::make_unique<RosData>();

  // Configure the codelet to tick periodically
  tickPeriodically();
}

void TfPublisher::stop() { ros_data_ = nullptr; }

void TfPublisher::tick() {
  if (ros::ok()) {
    // LOG_DEBUG("Publishing tfs");

    // Setup information needed for our transforms
    std::optional<isaac::Pose3d> robot_to_frame;
    const ros::Time ros_time = ros::Time::now();

    // Get all requested tfs
    for (const std::string &s : get_tf_frames()) {
      robot_to_frame =
          node()->pose().tryGet(get_tf_base_frame(), s, getTickTime());
      if (robot_to_frame) {
        ros_data_->tf_broadcaster.sendTransform(
            tf::StampedTransform(pose3d_to_transform(*robot_to_frame), ros_time,
                                 get_tf_base_frame(), s));
        // LOG_DEBUG("Published tf for: %s -> %s", get_tf_base_frame().c_str(),
        //           s.c_str());
      }
    }
  } else {
    LOG_ERROR("Lost connection to ROS master; should shut down");
  }
}

}  // namespace benchbot
