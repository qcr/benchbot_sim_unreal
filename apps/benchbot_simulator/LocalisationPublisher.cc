#include "LocalisationPublisher.hh"
#include "helpers.hh"

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "ros/callback_queue.h"
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"

namespace benchbot {

struct LocalisationPublisher::RosData {
  ros::NodeHandle nh;
  ros::ServiceServer service;
  ros::Publisher pub;
  tf::TransformBroadcaster tf_broadcaster;
  ros::CallbackQueue callback_queue;
};

void LocalisationPublisher::start() {
  // Start a ROS node & delegate SIGINT handling to Isaac
  ros::M_string args;
  if (!ros::isInitialized()) {
    ros::init(args, "localisation_publisher",
              ros::init_options::NoSigintHandler);
  }

  // Initialise all of the ROS data we are going to need
  ros_data_ = std::make_unique<RosData>();
  ros_data_->nh.setCallbackQueue(&(ros_data_->callback_queue));
  ros_data_->service = ros_data_->nh.advertiseService(
      "/reset_initial_pose", &LocalisationPublisher::callbackResetInitialPose,
      this);
  ros_data_->pub =
      ros_data_->nh.advertise<geometry_msgs::PoseWithCovarianceStamped>(
          get_initial_pose_channel_name(), 1);

  // Configure the codelet to tick periodically
  tickPeriodically();
}

void LocalisationPublisher::stop() { ros_data_ = nullptr; }

void LocalisationPublisher::tick() {
  if (ros::ok()) {
    const ros::Time ros_time = ros::Time::now();

    // Get the world_to_init pose if we haven't already (it shouldn't change??)
    if (!world_to_init_) {
      world_to_init_ = node()->pose().tryGet(
          get_gt_world_frame(), get_gt_init_frame(), getTickTime());
      if (!world_to_init_) LOG_ERROR("FAILED TO GET INITIAL POSE");
    }

    // Publish ground truth localisation if mode requested
    if (get_ground_truth_mode()) {
      // Attempt to get poses required for the
      const std::optional<isaac::Pose3d> noisy_odom(node()->pose().tryGet(
          get_noisy_odom_frame(), get_noisy_robot_frame(), getTickTime())),
          init_to_robot(node()->pose().tryGet(
              get_gt_init_frame(), get_gt_robot_frame(), getTickTime()));

      // Publish the TF if possible
      if (noisy_odom && init_to_robot) {
        ros_data_->tf_broadcaster.sendTransform(tf::StampedTransform(
            pose3d_to_transform((*world_to_init_) * (*init_to_robot) *
                                noisy_odom->inverse()),
            ros_time, get_tf_map_frame(), get_tf_odom_frame()));
      }
    }

    // Tick the callback queue
    ros_data_->callback_queue.callAvailable();
  } else {
    LOG_ERROR("Lost connection to ROS master; should shut down");
  }
}

bool LocalisationPublisher::callbackResetInitialPose(
    std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp) {
  geometry_msgs::PoseWithCovarianceStamped p;
  p.header.frame_id = get_tf_map_frame();
  p.pose.pose = pose3d_to_pose(*world_to_init_);
  p.pose.covariance[0] = 0.5 * 0.5;                   // ROS AMCL default...
  p.pose.covariance[7] = 0.5 * 0.5;                   // ROS AMCL default...
  p.pose.covariance[35] = M_PI / 12.0 * M_PI / 12.0;  // ROS AMCL default...
  ros_data_->pub.publish(p);
  return true;
}

}  // namespace benchbot
