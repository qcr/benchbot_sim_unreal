#include "RgbPublisher.hh"

#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "messages/camera.hpp"
#include "ros/ros.h"

namespace benchbot {

struct RgbPublisher::RosData {
  ros::NodeHandle nh;
  image_transport::Publisher pub;
  image_transport::ImageTransport it = image_transport::ImageTransport(nh);
};

void RgbPublisher::start() {
  // Start a ROS node & delegate SIGINT handling to Isaac
  ros::M_string args;
  if (!ros::isInitialized()) {
    ros::init(args, "rgb_publisher", ros::init_options::NoSigintHandler);
  }

  // Initialise all of the ROS data we are going to need
  ros_data_ = std::make_unique<RosData>();
  ros_data_->it = image_transport::ImageTransport(ros_data_->nh);
  ros_data_->pub = ros_data_->it.advertise(get_rgb_channel_name(), 2);

  // Configure the codelet to only tick when we receive a new camera message
  // from the simulator
  tickOnMessage(rx_camera_rgb());
}

void RgbPublisher::stop() {
  ros_data_->pub.shutdown();
  ros_data_ = nullptr;
}

void RgbPublisher::tick() {
  if (ros::ok()) {
    LOG_DEBUG("Received RGB from Isaac; passing to ROS");

    // Received a message, cache time ASAP
    ros::Time msg_time = ros::Time::now();

    // Attempt to get image data out of the message
    auto rgb_proto = rx_camera_rgb().getProto();
    isaac::ImageConstView3ub rgb_isaac;
    if (!FromProto(rgb_proto.getImage(), rx_camera_rgb().buffers(),
                   rgb_isaac)) {
      LOG_ERROR("Failed to get an image from the proto");
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
    ros_data_->pub.publish(rgb_ros);
  } else {
    LOG_ERROR("Lost connection to ROS master; should shut down");
  }
}

}  // namespace benchbot
