#include "odom_publisher.hh"

#include "helpers.hh"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "messages/math.hpp"
#include "engine/core/math/pose3.hpp"

template <typename T>
inline int _sign(const T num) {
  return (num > 0) ? 1 : ((num < 0) ? -1 : 0);
}

namespace benchbot {

struct OdomPublisher::RosData {
  ros::NodeHandle nh;
  ros::Publisher pub;
  tf::TransformBroadcaster tf_broadcaster;
};

void OdomPublisher::start() {
  // Start our monolothic ROS node if it doesn't already exist
  start_ros_node();

  // Initialise all of the ROS data we are going to need
  ros_data_ = std::make_unique<RosData>();
  ros_data_->pub =
      ros_data_->nh.advertise<nav_msgs::Odometry>(get_odom_channel_name(), 10);

  // Configure the codelet to only tick when we receive a new odom message from
  // the simulator
  tickOnMessage(rx_robot_odom());
}

void OdomPublisher::stop() {
  ros_data_->pub.shutdown();
  ros_data_ = nullptr;
}

void OdomPublisher::tick() {
  if (ros::ok()) {
    // LOG_DEBUG("Received odom from Isaac; passing to ROS");

    // Received a message, cache time ASAP
    ros::Time msg_time = ros::Time::now();

    // Send a transform ROS message from the Isaac message
    auto odom_proto = rx_robot_odom().getProto();
    tf::Transform odom_tf = tf::Transform(
        tf::createQuaternionFromYaw(
            std::abs(std::acos(
                odom_proto.getOdomTRobot().getRotation().getQ().getX())) *
            _sign(std::asin(
                odom_proto.getOdomTRobot().getRotation().getQ().getY()))),
        tf::Vector3(odom_proto.getOdomTRobot().getTranslation().getX(),
                    odom_proto.getOdomTRobot().getTranslation().getY(), 0));
    ros_data_->tf_broadcaster.sendTransform(
        tf::StampedTransform(odom_tf, msg_time, get_ros_odom_frame(),
                             get_ros_robot_frame()));
    // std::cout << get_ros_robot_frame() << std::endl;

    // Send an Odometry message
    nav_msgs::Odometry odom_ros;
    odom_ros.header.stamp = msg_time;
    odom_ros.header.frame_id = odom_proto.getOdometryFrame();
    odom_ros.child_frame_id = odom_proto.getRobotFrame();

    odom_ros.pose.pose.position.x = odom_tf.getOrigin().x();
    odom_ros.pose.pose.position.y = odom_tf.getOrigin().y();

    tf::quaternionTFToMsg(odom_tf.getRotation(),
                          odom_ros.pose.pose.orientation);

    odom_ros.twist.twist.linear.x = odom_proto.getSpeed().getX();
    odom_ros.twist.twist.linear.y = odom_proto.getSpeed().getY();
    odom_ros.twist.twist.angular.z = odom_proto.getAngularSpeed();
    // TODO covariance???
    ros_data_->pub.publish(odom_ros);

    // Publish map->odom in tf tree to ensure map->odom->robot gives gt_pose
    // Check if we have robot pose gt data required for publishing odom pose in gt tree
    const std::optional<isaac::Pose3d> gt_world_to_robot(node()->pose().tryGet(
            get_gt_world_frame(), get_gt_robot_frame(), getTickTime()));
    if (!gt_world_to_robot) {
      return;
    }
    // Calculate and publish transform
    // TODO could probably be condensed
    isaac::Pose2d odom_to_robot_2d = isaac::FromProto(odom_proto.getOdomTRobot());
    isaac::Pose3d odom_to_robot_3d = isaac::Pose3d::FromPose2XY(odom_to_robot_2d);
    ros_data_->tf_broadcaster.sendTransform(tf::StampedTransform(
      pose3d_to_transform(*gt_world_to_robot * odom_to_robot_3d.inverse()),
      msg_time, get_ros_world_frame(), get_ros_odom_frame()));

  } else {
    LOG_ERROR("ROS is not ok");
  }
}

}  // namespace benchbot
