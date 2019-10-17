#include "helpers.hh"
#include "localisation_publisher.hh"

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
    // const ros::Time ros_time = ros::Time::now();
    // If world to odom doesn't exist, it should be set at least once
    const std::optional<isaac::Pose3d> noisy_odom_to_robot(
        node()->pose().tryGet(get_noisy_odom_frame(), get_noisy_robot_frame(),
                              getTickTime())),
        gt_world_to_robot(node()->pose().tryGet(
            get_gt_world_frame(), get_gt_robot_frame(), getTickTime()));
    if (!node()->pose().tryGet(get_gt_world_frame(), get_noisy_odom_frame(),
                               getTickTime())) {
      node()->pose().set(get_gt_world_frame(), get_noisy_odom_frame(),
                         *gt_world_to_robot * noisy_odom_to_robot->inverse(),
                         getTickTime());
      LOG_WARNING("No '%s' to '%s' found, so we set it...",
                  get_gt_world_frame().c_str(), get_noisy_odom_frame().c_str());
    }

    const std::optional<isaac::Pose3d> noisy_world_to_robot(
        node()->pose().tryGet(get_gt_world_frame(), get_noisy_robot_frame(),
                              getTickTime()));
    LOG_INFO("GT: \n(%f,%f,%f)+(%f,%f,%f) \nvs Noise: \n(%f,%f,%f)+(%f,%f,%f)",
             gt_world_to_robot->translation.x(),
             gt_world_to_robot->translation.y(),
             gt_world_to_robot->translation.z(),
             gt_world_to_robot->rotation.eulerAnglesRPY().x(),
             gt_world_to_robot->rotation.eulerAnglesRPY().y(),
             gt_world_to_robot->rotation.eulerAnglesRPY().z(),
             noisy_world_to_robot->translation.x(),
             noisy_world_to_robot->translation.y(),
             noisy_world_to_robot->translation.z(),
             noisy_world_to_robot->rotation.eulerAnglesRPY().x(),
             noisy_world_to_robot->rotation.eulerAnglesRPY().y(),
             noisy_world_to_robot->rotation.eulerAnglesRPY().z());

    // Publish ground truth localisation if mode requested
    if (get_ground_truth_mode()) {
      // // Attempt to get poses required for the
      // const std::optional<isaac::Pose3d> noisy_odom(node()->pose().tryGet(
      //     get_noisy_odom_frame(), get_noisy_robot_frame(), getTickTime())),
      //     init_to_robot(node()->pose().tryGet(
      //         get_gt_init_frame(), get_gt_robot_frame(), getTickTime()));

      // // Publish the TF if possible
      // if (noisy_odom && init_to_robot) {
      //   ros_data_->tf_broadcaster.sendTransform(tf::StampedTransform(
      //       pose3d_to_transform((*world_to_init_) * (*init_to_robot) *
      //                           noisy_odom->inverse()),
      //       ros_time, get_tf_map_frame(), get_tf_odom_frame()));
      // }
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
  // Get the robot's current Ground Truth pose
  const std::optional<isaac::Pose3d> gt_world_to_robot(node()->pose().tryGet(
      get_gt_world_frame(), get_gt_robot_frame(), getTickTime())),
      noisy_odom_to_robot(node()->pose().tryGet(
          get_noisy_odom_frame(), get_noisy_robot_frame(), getTickTime()));
  if (!gt_world_to_robot || !noisy_odom_to_robot) return false;

  // Reset to the Ground Truth in ROS
  p.header.frame_id = get_tf_map_frame();
  p.pose.pose = pose3d_to_pose(*gt_world_to_robot);
  p.pose.covariance[0] = 0.5 * 0.5;                   // ROS AMCL default...
  p.pose.covariance[7] = 0.5 * 0.5;                   // ROS AMCL default...
  p.pose.covariance[35] = M_PI / 12.0 * M_PI / 12.0;  // ROS AMCL default...
  ros_data_->pub.publish(p);

  // Reset odom to Ground Truth in Isaac
  node()->pose().set(get_gt_world_frame(), get_noisy_odom_frame(),
                     *gt_world_to_robot * noisy_odom_to_robot->inverse(),
                     getTickTime());
  return true;
}

}  // namespace benchbot
