#include "tf_publisher.hh"

#include "helpers.hh"
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"

namespace benchbot {

struct TfPublisher::RosData {
  ros::NodeHandle nh;
  tf::TransformBroadcaster tf_broadcaster;
};

void TfPublisher::start() {
  // Start our monolothic ROS node if it doesn't already exist
  start_ros_node();

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

    // Determine if the ros base frame is the same as isaac (default) or defined by user
    std::string ros_base_frame = (get_ros_base_frame().empty()) ? get_isaac_base_frame() : get_ros_base_frame();
    
    // Get all requested tfs
    for (unsigned int idx = 0; idx < get_isaac_child_frames().size(); idx++){
      robot_to_frame =
          node()->pose().tryGet(get_isaac_base_frame(), get_isaac_child_frames()[idx], getTickTime());
      
      // Determine if the ros child frame is the same as isaac (default) or if it has been defined by user
      // NOTE if a user gives fewer names than there are available, system will assume default after 
      // all child frame names have been used up in order (empty string indicates use default)
      std::string ros_child_frame;
      if (idx < get_ros_child_frames().size()){
        ros_child_frame = (get_ros_child_frames()[idx].empty()) ? get_isaac_child_frames()[idx] : get_ros_child_frames()[idx];
      } else {
        ros_child_frame = get_isaac_child_frames()[idx];
      }
      
      // Publish the tf
      if (robot_to_frame) {
        ros_data_->tf_broadcaster.sendTransform(
            tf::StampedTransform(pose3d_to_transform(*robot_to_frame), ros_time,
                                 ros_base_frame, ros_child_frame));
        // LOG_DEBUG("Published tf for: %s -> %s", get_tf_base_frame().c_str(),
        //           s.c_str());
      }
    }
  } else {
    LOG_ERROR("Lost connection to ROS master; should shut down");
  }
}

}  // namespace benchbot
