#include "segment_publisher.hh"

#include "cv_bridge/cv_bridge.h"
#include "helpers.hh"
#include "image_transport/image_transport.h"
#include "messages/camera.hpp"
#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "std_msgs/String.h"

namespace benchbot {

struct SegmentPublisher::RosData {
  ros::NodeHandle nh;
  image_transport::Publisher pub_label;
  image_transport::Publisher pub_instance;
  ros::Publisher pub_label_list;
  ros::Publisher pub_info;
  image_transport::ImageTransport it = image_transport::ImageTransport(nh);
  sensor_msgs::CameraInfo info_msg;
};

void SegmentPublisher::start() {
  // Start our monolothic ROS node if it doesn't already exist
  start_ros_node();

  // Initialise all of the ROS data we are going to need
  ros_data_ = std::make_unique<RosData>();
  ros_data_->it = image_transport::ImageTransport(ros_data_->nh);
  ros_data_->pub_label = ros_data_->it.advertise(get_segment_label_channel_name(), 2);
  ros_data_->pub_instance = ros_data_->it.advertise(get_segment_instance_channel_name(), 2);
  ros_data_->pub_label_list = ros_data_->nh.advertise<std_msgs::String>(
    get_segment_label_list_channel_name(), 2);
  ros_data_->pub_info = ros_data_->nh.advertise<sensor_msgs::CameraInfo>(
      get_info_channel_name(), 1);

  // Setup our static camera_info message
  ros_data_->info_msg.header.frame_id = get_segment_frame_name();
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
  tickOnMessage(rx_camera_segment());
}

void SegmentPublisher::stop() {
  ros_data_->pub_label.shutdown();
  ros_data_->pub_instance.shutdown();
  ros_data_->pub_label_list.shutdown();
  ros_data_->pub_info.shutdown();
  ros_data_ = nullptr;
}

void SegmentPublisher::tick() {
  if (ros::ok()) {
    LOG_DEBUG("Received Segment from Isaac; passing to ROS");

    // Received a message, cache time ASAP
    ros::Time msg_time = ros::Time::now();

    // Attempt to get image data out of the message
    auto segment_proto = rx_camera_segment().getProto();

    isaac::ImageConstView1ub label_isaac;
    if (!FromProto(segment_proto.getLabelImage(), rx_camera_segment().buffers(), label_isaac)) {
      LOG_ERROR("Failed to get label image from the proto");
    };
    // NOTE I am not certain if a different buffer is needed for instance and label images
    isaac::ImageConstView1ui16 instance_isaac;
    if (!FromProto(segment_proto.getInstanceImage(), rx_camera_segment().buffers(), instance_isaac)) {
      LOG_ERROR("Failed to get label image from the proto");
    };
    // Use OpenCV to get images that ROS will understand
    cv::Mat label_cv = cv::Mat(label_isaac.rows(), label_isaac.cols(), CV_8UC1,
                                const_cast<void *>(static_cast<const void *>(
                                label_isaac.data().pointer())));
    cv::Mat instance_cv = cv::Mat(instance_isaac.rows(), instance_isaac.cols(), CV_16UC1,
                                   const_cast<void *>(static_cast<const void *>(
                                    instance_isaac.data().pointer())));

    // Form & publish the ROS Image messages to ROS
    sensor_msgs::ImagePtr label_ros = 
      cv_bridge::CvImage(std_msgs::Header(), "8UC1", label_cv).toImageMsg();
    sensor_msgs::ImagePtr instance_ros = 
      cv_bridge::CvImage(std_msgs::Header(), "16UC1", instance_cv).toImageMsg();
    label_ros->header.stamp = msg_time;
    label_ros->header.frame_id = get_segment_frame_name();
    instance_ros->header.stamp = msg_time;
    instance_ros->header.frame_id = get_segment_frame_name();

    // Create a single string of all labels in label list, separated by ':'
    std::string label_list_string;
    for (auto label_proto : segment_proto.getLabels()) {
      if (label_list_string.length() == 0){
        label_list_string = label_proto.getName().cStr();
      } else {
        label_list_string = label_list_string.append(":").append(
          label_proto.getName().cStr());
      }
    }
    // Publish label list to ros
    std_msgs::String label_list_msg;
    label_list_msg.data = label_list_string;
    ros_data_->pub_label_list.publish(label_list_msg);

    // Publish static camera_info for the RGB sensor
    ros_data_->info_msg.header.stamp = msg_time;
    ros_data_->pub_info.publish(ros_data_->info_msg);
  } else {
    LOG_ERROR("Lost connection to ROS master; should shut down");
  }
}
}  // namespace benchbot
