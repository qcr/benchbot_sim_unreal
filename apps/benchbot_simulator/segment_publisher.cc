#include "segment_publisher.hh"

#include "cv_bridge/cv_bridge.h"
#include "helpers.hh"
#include "image_transport/image_transport.h"
#include "messages/camera.hpp"
#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "std_msgs/String.h"
#include "benchbot_msgs/isaac_segment_img.h"

namespace benchbot {

template<typename type>
std::vector<type> unique(cv::Mat in) {
    assert(in.channels() == 1 && "This implementation is only for single-channel images");
    auto begin = in.begin<type>(), end = in.end<type>();
    auto last = std::unique(begin, end);    // remove adjacent duplicates to reduce size
    std::sort(begin, last);                 // sort remaining elements
    last = std::unique(begin, last);        // remove duplicates
    return std::vector<type>(begin, last);
}

struct SegmentPublisher::RosData {
  ros::NodeHandle nh;
  ros::Publisher pub_info;
  ros::Publisher pub_segment;
  sensor_msgs::CameraInfo info_msg;
};

void SegmentPublisher::start() {
  // Start our monolothic ROS node if it doesn't already exist
  start_ros_node();

  // Initialise all of the ROS data we are going to need
  ros_data_ = std::make_unique<RosData>();
  ros_data_->pub_info = ros_data_->nh.advertise<sensor_msgs::CameraInfo>(
      get_info_channel_name(), 1);
  ros_data_->pub_segment = ros_data_->nh.advertise<isaac_ros_msgs::isaac_segment_img>(
      get_segment_channel_name(), 2);

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
  ros_data_->pub_info.shutdown();
  ros_data_->pub_segment.shutdown();
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
      LOG_ERROR("Failed to get instance image from the proto");
    };
    // Use OpenCV to get images that ROS will understand
    cv::Mat label_cv = cv::Mat(label_isaac.rows(), label_isaac.cols(), CV_8UC1,
                                const_cast<void *>(static_cast<const void *>(
                                label_isaac.data().pointer())));
    cv::Mat instance_cv_original = cv::Mat(instance_isaac.rows(), instance_isaac.cols(), CV_16UC1,
                                   const_cast<void *>(static_cast<const void *>(
                                    instance_isaac.data().pointer())));
    
    // Change instance image so that each instance gets its own value
    // This will be made up of both the class ID and the instance ID
    // Instance pixel value: class_id x 1000 + instance_id
    // NOTE this only works for fewer than 1000 instances and 65 classes
    cv::Mat instance_cv = cv::Mat::zeros(instance_cv_original.rows, 
                                         instance_cv_original.cols,
                                         CV_16UC1);
    auto instance_ids_original = unique<uint16_t>(instance_cv_original.clone());
    auto class_ids = unique<uint8_t>(label_cv.clone());
    // Keep track of number of instances of each class
    std::vector<int> class_inst_counts(class_ids.size(), 0);

    // Go through all instance ids in the original
    for (auto inst_id = instance_ids_original.begin(); inst_id != instance_ids_original.end(); ++inst_id)
    {
      // Skip instance id 0 (unlabelled)
      if (*inst_id > 0)
      {
        // Get the mask of the current instance id
        cv::Mat inst_mask;
        cv::inRange(instance_cv_original, *inst_id, *inst_id, inst_mask);

        // Make mask of class image within this instance id
        // This gives us all classes with this original instance id
        cv::Mat masked_label_image;
        label_cv.copyTo(masked_label_image, inst_mask);
        
        // Go through all class ids in the masked image
        auto masked_class_ids = unique<uint8_t>(masked_label_image.clone());
        for (auto cls_id = masked_class_ids.begin(); cls_id != masked_class_ids.end(); ++cls_id)
        {
          // Skip class id 0 (unlabelled)
          if ((uint16_t)*cls_id > 0)
          {
            // Increase count of instances for given class by 1
            int cls_count_idx = std::distance(class_ids.begin(),
                                              std::find(class_ids.begin(),
                                                        class_ids.end(),
                                                        *cls_id));

            class_inst_counts[cls_count_idx]++;

            // Define new instance id
            uint16_t new_inst_id = (uint16_t)*cls_id * 1000 + class_inst_counts[cls_count_idx];

            // Define the mask for the instance
            cv::Mat cls_inst_mask;
            cv::inRange(masked_label_image, *cls_id, *cls_id, cls_inst_mask);

            // Set values in the final instance image to the new instance id
            instance_cv.setTo(new_inst_id, cls_inst_mask);
          }
        }
      }
    }

    // Form the ROS message
    benchbot_msgs::isaac_segment_img segment_msg;

    // Setup Image message components
    sensor_msgs::ImagePtr label_ros = 
      cv_bridge::CvImage(std_msgs::Header(), "8UC1", label_cv).toImageMsg();
    sensor_msgs::ImagePtr instance_ros = 
      cv_bridge::CvImage(std_msgs::Header(), "16UC1", instance_cv).toImageMsg();
    label_ros->header.stamp = msg_time;
    label_ros->header.frame_id = get_segment_frame_name();
    instance_ros->header.stamp = msg_time;
    instance_ros->header.frame_id = get_segment_frame_name();

    segment_msg.class_segment_img = *label_ros;
    segment_msg.instance_segment_img = *instance_ros;

    // Setup class names and class ids
    // Note class_ids need to be increased by one to match image
    // Original missed background "unlabelled" class.
    for (auto label_proto : segment_proto.getLabels())
    {
      segment_msg.class_names.push_back(label_proto.getName().cStr());
      segment_msg.class_ids.push_back(label_proto.getIndex() + 1);
    }

    // Publish the segment camera message
    ros_data_->pub_segment.publish(segment_msg);

    // Publish static camera_info for the RGB sensor
    ros_data_->info_msg.header.stamp = msg_time;
    ros_data_->pub_info.publish(ros_data_->info_msg);
  } else {
    LOG_ERROR("Lost connection to ROS master; should shut down");
  }
}
}  // namespace benchbot
