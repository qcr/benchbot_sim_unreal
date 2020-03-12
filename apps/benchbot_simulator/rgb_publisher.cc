#include "rgb_publisher.hh"

#include "cv_bridge/cv_bridge.h"
#include "helpers.hh"
#include "image_transport/image_transport.h"
#include "messages/camera.hpp"
#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"

namespace benchbot {

struct RgbPublisher::RosData {
  ros::NodeHandle nh;
  image_transport::Publisher pub_rgb;
  ros::Publisher pub_info;
  image_transport::ImageTransport it = image_transport::ImageTransport(nh);

  sensor_msgs::CameraInfo info_msg;
};

void RgbPublisher::start() {
  // Start our monolothic ROS node if it doesn't already exist
  start_ros_node();

  // Initialise all of the ROS data we are going to need
  ros_data_ = std::make_unique<RosData>();
  ros_data_->it = image_transport::ImageTransport(ros_data_->nh);
  ros_data_->pub_rgb = ros_data_->it.advertise(get_rgb_channel_name(), 2);
  ros_data_->pub_info = ros_data_->nh.advertise<sensor_msgs::CameraInfo>(
      get_info_channel_name(), 1);

  // Setup our static camera_info message
  ros_data_->info_msg.header.frame_id = get_rgb_frame_name();
  ros_data_->info_msg.width = get_info_center_x() * 2;
  ros_data_->info_msg.height = get_info_center_y() * 2;
  ros_data_->info_msg.K = {get_info_fx(),
                           0,
                           get_info_center_x(),
                           0,
                           get_info_fy(),
                           get_info_center_y(),
                           0,
                           0,
                           1};
  ros_data_->info_msg.P = {get_info_fx(),
                           0,
                           get_info_center_x(),
                           0,
                           0,
                           get_info_fy(),
                           get_info_center_y(),
                           0,
                           0,
                           0,
                           1,
                           0};

  // Configure the codelet to only tick when we receive a new camera message
  // from the simulator
  tickOnMessage(rx_camera_rgb());
}

void RgbPublisher::stop() {
  ros_data_->pub_rgb.shutdown();
  ros_data_->pub_info.shutdown();
  ros_data_ = nullptr;
}

void RgbPublisher::tick() {
  if (ros::ok()) {
    // LOG_DEBUG("Received RGB from Isaac; passing to ROS");

    // Received a message, cache time ASAP
    ros::Time msg_time = ros::Time::now();

    // Attempt to get image data out of the message
    auto rgb_proto = rx_camera_rgb().getProto();
    isaac::ImageConstView3ub rgb_isaac;
    if (!FromProto(rgb_proto.getImage(), rx_camera_rgb().buffers(),
                   rgb_isaac)) {
      LOG_ERROR("Failed to get an RGB image from the proto");
    }

    // Use OpenCv to get an image that ROS will understand
    cv::Mat rgb_cv = cv::Mat(rgb_isaac.rows(), rgb_isaac.cols(), CV_8UC3,
                             const_cast<void *>(static_cast<const void *>(
                                 rgb_isaac.data().pointer())));
    cv::cvtColor(rgb_cv, rgb_cv, cv::COLOR_RGB2BGR);

    // Form & publish the ROS Image message to ROS
    sensor_msgs::ImagePtr rgb_ros =
        cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgb_cv).toImageMsg();
    rgb_ros->header.stamp = msg_time;
    rgb_ros->header.frame_id = get_rgb_frame_name();
    ros_data_->pub_rgb.publish(rgb_ros);

    // Publish static camera_info for the RGB sensor
    ros_data_->info_msg.header.stamp = msg_time;
    ros_data_->pub_info.publish(ros_data_->info_msg);
  } else {
    LOG_ERROR("Lost connection to ROS master; should shut down");
  }
}

}  // namespace benchbot
