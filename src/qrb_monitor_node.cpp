// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_benchmark/qrb_monitor_node.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using std::placeholders::_1;

namespace qrb_ros
{
namespace benchmark
{

QrbMonitorNode::QrbMonitorNode(const rclcpp::NodeOptions & options)
: ros2_benchmark::MonitorNode("QrbMonitorNode", options)
{
  RCLCPP_INFO(this->get_logger(), "QrbMonitorNode create");
  create_monitor_subscriber();
}

void QrbMonitorNode::create_monitor_subscriber()
{
  RCLCPP_INFO(this->get_logger(), "Monitor data format=\"%s\"", monitor_data_format_.c_str());
  // Create transport type subscriber
  if (monitor_data_format_.compare("qrb_ros/transport/type/Image") == 0) {
    return create_message_subscriber<qrb_ros::transport::type::Image>();
  } else if (monitor_data_format_.compare("qrb_ros/transport/type/Imu") == 0) {
    return create_message_subscriber<qrb_ros::transport::type::Imu>();
  } else if (monitor_data_format_.compare("dmabuf_transport/type/Image") == 0) {
    return create_message_subscriber<dmabuf_transport::type::Image>();
  } else if (monitor_data_format_.compare("dmabuf_transport/type/PointCloud2") == 0) {
    return create_message_subscriber<dmabuf_transport::type::PointCloud2>();
  }

  // Create ros message type subscriber
  #define CREATE_ROS_MESSAGE_SUBSCRIBER(ROS_MESSAGE) \
  if (rosidl_generator_traits::name<ROS_MESSAGE>() == monitor_data_format_) { \
    return create_message_subscriber<ROS_MESSAGE>(); \
  } \

  CREATE_ROS_MESSAGE_SUBSCRIBER(sensor_msgs::msg::Image)
  CREATE_ROS_MESSAGE_SUBSCRIBER(sensor_msgs::msg::CompressedImage)
  CREATE_ROS_MESSAGE_SUBSCRIBER(qrb_ros_tensor_list_msgs::msg::TensorList)

  // Create generic type message subscriber(SerializedMessage)
  CreateGenericTypeMonitorSubscriber();
}

template<typename ROSMessageType>
void QrbMonitorNode::create_message_subscriber()
{
  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;

  std::function<void(std::shared_ptr<ROSMessageType>)> subscriber_callback =
    std::bind(&QrbMonitorNode::ros_message_subscriber_callback<ROSMessageType>,
    this, std::placeholders::_1);

  monitor_sub_ = create_subscription<ROSMessageType>("output", ros2_benchmark::kQoS,
    subscriber_callback, subscription_options);

  RCLCPP_INFO(this->get_logger(), "Create a qrb monitor subscriber: format=\"%s\", topic=\"%s\"",
    monitor_data_format_.c_str(), monitor_sub_->get_topic_name());
}

template<typename ROSMessageType>
void QrbMonitorNode::ros_message_subscriber_callback(
  const std::shared_ptr<ROSMessageType> ros_msg)
{
  std::lock_guard<std::mutex> lock(is_monitoring_mutex_);
  if (!is_monitoring_) {
    return;
  }

  // Record end time
  uint32_t key;
  if (!revise_timestamps_as_message_ids_) {
    RecordEndTimestampAutoKey();
    key = end_timestamps_.size();
  } else {
    RecordEndTimestamp(ros_msg->header.stamp.sec);
    key = ros_msg->header.stamp.sec;
  }

  // Record start time
  if (record_start_timestamps_) {
    std::chrono::time_point<std::chrono::system_clock> msg_timestamp(
      std::chrono::seconds(ros_msg->header.stamp.sec) +
      std::chrono::nanoseconds(ros_msg->header.stamp.nanosec));
    RecordStartTimestamp(key, msg_timestamp);
  }
}

}  // namespace benchmark
}  // namespace qrb_ros

// Register QrbMonitorNode as a component
RCLCPP_COMPONENTS_REGISTER_NODE(qrb_ros::benchmark::QrbMonitorNode)