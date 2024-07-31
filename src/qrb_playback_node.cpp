// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_benchmark/qrb_playback_node.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using std::placeholders::_1;

namespace qrb_ros
{
namespace benchmark
{

size_t MessageBufferBase::get_buffer_size()
{
  return 0;
}

void MessageBufferBase::clear()
{

}


template<typename ROSMessageType>
size_t MessageBuffer<ROSMessageType>::get_buffer_size()
{
  return message_buffer.size();
}

template<typename ROSMessageType>
void MessageBuffer<ROSMessageType>::clear()
{
  message_buffer.clear();
}


QrbPlaybackNode::QrbPlaybackNode(const rclcpp::NodeOptions & options)
: ros2_benchmark::PlaybackNode("QrbPlaybackNode", options)
{
  RCLCPP_INFO(this->get_logger(), "QrbPlaybackNode create");
  if (data_formats_.empty()) {
    throw std::invalid_argument("Empty data_formats");
  }

  for (size_t format_index = 0; format_index < data_formats_.size(); format_index++) {
    create_playback_pubsub(format_index, data_formats_[format_index]);
  }
}

void QrbPlaybackNode::create_playback_pubsub(const size_t index, const std::string format)
{
  RCLCPP_INFO(this->get_logger(), "Data format=\"%s\", index=%ld", format.c_str(), index);

  // Create transport_type pubsub
  if (format.compare("qrb_ros/transport/type/Image") == 0) {
    pub_sub_types_[index] = PubSubType::QrbTransportType;
    return create_message_pubsub<qrb_ros::transport::type::Image>(index, format);
  } else if (format.compare("qrb_ros/transport/type/Imu") == 0) {
    pub_sub_types_[index] = PubSubType::QrbTransportType;
    return create_message_pubsub<qrb_ros::transport::type::Imu>(index, format);
  } else if (format.compare("dmabuf_transport/type/Image") == 0) {
    pub_sub_types_[index] = PubSubType::DmabufTransportType;
    return create_message_pubsub<dmabuf_transport::type::Image>(index, format);
  } else if (format.compare("dmabuf_transport/type/PointCloud2") == 0) {
    pub_sub_types_[index] = PubSubType::DmabufTransportType;
    return create_message_pubsub<dmabuf_transport::type::PointCloud2>(index, format);
  }

  // Create ros message type pubsub
  #define CREATE_ROS_MESSAGE_PUBSUB(ROS_MESSAGE) \
  if (rosidl_generator_traits::name<ROS_MESSAGE>() == format) { \
    pub_sub_types_[index] = PubSubType::RosMessaageType; \
    return create_message_pubsub<ROS_MESSAGE>(index, format); \
  } \

  CREATE_ROS_MESSAGE_PUBSUB(sensor_msgs::msg::Image)
  CREATE_ROS_MESSAGE_PUBSUB(sensor_msgs::msg::CompressedImage)
  CREATE_ROS_MESSAGE_PUBSUB(qrb_ros_tensor_list_msgs::msg::TensorList)

  //  Create generic_type type pubsub
  pub_sub_types_[index] = PubSubType::GenericType;
  ros2_benchmark::PlaybackNode::CreateGenericPubSub(format, index);
}

template<typename ROSMessageType>
void QrbPlaybackNode::create_message_pubsub(const size_t index, const std::string format)
{
  // Create ROSMessageType publisher
  rclcpp::PublisherOptions publisher_options;
  publisher_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
  auto publisher = this->create_publisher<ROSMessageType>(
    "input" + std::to_string(index), ros2_benchmark::kQoS, publisher_options);
  message_pubs_[index] = publisher;

  RCLCPP_INFO(this->get_logger(), "Create a publisher: index=%ld, format=\"%s\", topic=\"%s\"",
      index, format.c_str(), publisher->get_topic_name());

  // Create ROSMessageType Subscription
  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Enable;
  std::function<void(std::shared_ptr<ROSMessageType>)> subscriber_callback =
    std::bind(&QrbPlaybackNode::message_subscriber_callback<ROSMessageType>, this,
    std::placeholders::_1, index);

  auto subscriber = this->create_subscription<ROSMessageType>(
    "buffer/input" + std::to_string(index), ros2_benchmark::kBufferQoS,
    subscriber_callback, subscription_options);
  subs_[index] = subscriber;

  RCLCPP_INFO(this->get_logger(), "Create a subscriber: index=%ld, format=\"%s\", topic=\"%s\"",
      index, format.c_str(), subscriber->get_topic_name());

  // Create ROSMessageType message buffer
  std::shared_ptr<MessageBuffer<ROSMessageType>> message_buffer
      = std::make_shared<MessageBuffer<ROSMessageType>>();
  message_buffer->message_format = format;
  message_buffer_map_[index] = message_buffer;
}

template<typename ROSMessageType>
void QrbPlaybackNode::message_subscriber_callback(
  const std::shared_ptr<ROSMessageType> msg,
  const size_t index)
{
  RCLCPP_DEBUG(this->get_logger(), "message_subscriber_callback");

  // Get MessageBuffer<ROSMessageType>
  auto buffer_base = message_buffer_map_[index];
  std::shared_ptr<MessageBuffer<ROSMessageType>> msg_buffer
      = std::dynamic_pointer_cast<MessageBuffer<ROSMessageType>>(buffer_base);
  if(!msg_buffer) {
    RCLCPP_ERROR(this->get_logger(), "Message buffer is error, index=%ld", index);
    return;
  }

  // Check size
  size_t size = msg_buffer->message_buffer.size();
  if ((size >= max_size_) || (requested_buffer_length_ > 0 && size >= requested_buffer_length_))
  {
    RCLCPP_DEBUG(this->get_logger(), "Message buffer is full, drop the message, index=%ld", index);
    return;
  }

  // Save msg
  msg_buffer->message_buffer.push_back(msg);
  size += 1;
  RCLCPP_DEBUG(this->get_logger(),
    "Add the message to the buffer, current capacity (%ld/%ld), index=%ld",
    size, requested_buffer_length_, index);

  // Add timestamps
  if (record_data_timeline_) {
    int64_t time_ns = this->get_clock()->now().nanoseconds();
    size_t msg_index = size - 1;
    AddToTimestampsToMessagesMap(time_ns, index, msg_index);
  }
}

bool QrbPlaybackNode::AreBuffersFull() const
{
  RCLCPP_DEBUG(this->get_logger(), "AreBuffersFull");

  if (record_data_timeline_) {
    return false;
  }

  if (requested_buffer_length_ == 0) {
    if ((0 < timestamps_to_messages_map_.size()) &&
      (GetTimestampsToMessagesCount() == GetRecordedMessageCount())) {
      return true;
    }
    return false;
  }

  // Check msg buffer size
  for (size_t index = 0; index < pub_sub_types_.size(); index++) {
    switch (pub_sub_types_.at(index)) {
      case PubSubType::QrbTransportType:
      case PubSubType::DmabufTransportType:
      case PubSubType::RosMessaageType:
        if (message_buffer_map_.at(index)->get_buffer_size() < requested_buffer_length_) {
          return false;
        }
        break;
      case PubSubType::GenericType:
        if (serialized_msg_buffers_.at(index).size() < requested_buffer_length_) {
          return false;
        }
        break;
      default:
        return false;
    }
  }

  return true;
}

void QrbPlaybackNode::ClearBuffers()
{
  for (size_t index = 0; index < pub_sub_types_.size(); index++) {
    switch (pub_sub_types_.at(index)) {
      case PubSubType::QrbTransportType:
      case PubSubType::DmabufTransportType:
      case PubSubType::RosMessaageType:
        message_buffer_map_.at(index)->clear();
        break;
      case PubSubType::GenericType:
        serialized_msg_buffers_.at(index).clear();
        break;
    }
  }

  // Clear timestamps messages map
  timestamps_to_messages_map_.clear();
}

uint64_t QrbPlaybackNode::GetRecordedMessageCount() const
{
  uint64_t msg_count = 0;
  for (const auto & message_buffer : message_buffer_map_) {
    msg_count += message_buffer.second->get_buffer_size();
  }
  for (const auto & serialized_msg_buffer : serialized_msg_buffers_) {
    msg_count += serialized_msg_buffer.second.size();
  }
  return msg_count;
}

uint64_t QrbPlaybackNode::GetRecordedMessageCount(size_t pub_index) const
{
  switch (pub_sub_types_.at(pub_index)) {
    case PubSubType::GenericType:
      return serialized_msg_buffers_.at(pub_index).size();
    case PubSubType::QrbTransportType:
    case PubSubType::DmabufTransportType:
    case PubSubType::RosMessaageType:
      return message_buffer_map_.at(pub_index)->get_buffer_size();
    default:
      return 0;
  }
}

bool QrbPlaybackNode::PublishMessage(
  const size_t pub_index,
  const size_t message_index,
  const std::optional<std_msgs::msg::Header> & header = std::nullopt)
{
  // Publish serialized message
  if (pub_sub_types_[pub_index] == PubSubType::GenericType) {
    return ros2_benchmark::PlaybackNode::PublishMessage(pub_index, message_index, header);
  }

  // Check message_index
  size_t size = message_buffer_map_.at(pub_index)->get_buffer_size();
  if (message_index >= size) {
    RCLCPP_ERROR(get_logger(), "Failed to publish message, message index: %ld, " \
      " publishre index: %ld, and total recorded messages count: %ld",
      message_index, pub_index, size);
    return false;
  }

  // Publish message<ROSMessageType>
  std::string format = message_buffer_map_.at(pub_index)->message_format;
  switch (pub_sub_types_[pub_index]) {
    case PubSubType::QrbTransportType:
    case PubSubType::DmabufTransportType:
      if (format.compare("qrb_ros/transport/type/Image") == 0) {
        return publish_message<qrb_ros::transport::type::Image>(pub_index, message_index, header);
      } else if (format.compare("qrb_ros/transport/type/Imu") == 0) {
        return publish_message<qrb_ros::transport::type::Imu>(pub_index, message_index, header);
      } else if (format.compare("dmabuf_transport/type/Image") == 0) {
        return publish_message<dmabuf_transport::type::Image>(pub_index, message_index, header);
      } else if (format.compare("dmabuf_transport/type/PointCloud2") == 0) {
        return publish_message<dmabuf_transport::type::PointCloud2>(
          pub_index, message_index, header);
      }
      break;
    case PubSubType::RosMessaageType:
      #define PUBLISH_ROS_MESSAGE(ROSMessageType) \
      if (rosidl_generator_traits::name<ROSMessageType>() == format) { \
        return publish_message<ROSMessageType>(pub_index, message_index, header); \
      } \

      PUBLISH_ROS_MESSAGE(sensor_msgs::msg::Image)
      PUBLISH_ROS_MESSAGE(sensor_msgs::msg::CompressedImage)
      PUBLISH_ROS_MESSAGE(qrb_ros_tensor_list_msgs::msg::TensorList)
      break;
    default:
      return false;
  }
  return false;
}

size_t QrbPlaybackNode::GetPublisherCount() const
{
  return message_pubs_.size() + generic_pubs_.size();
}

template<typename ROSMessageType>
bool QrbPlaybackNode::publish_message(
  const size_t pub_index,
  const size_t message_index,
  const std::optional<std_msgs::msg::Header> & header)
{
  // Get message from MessageBuffer<ROSMessageType>
  auto buffer_base = message_buffer_map_[pub_index];
  std::shared_ptr<MessageBuffer<ROSMessageType>> msg_buffer
      = std::dynamic_pointer_cast<MessageBuffer<ROSMessageType>>(buffer_base);
  if(!msg_buffer) {
    RCLCPP_ERROR(get_logger(), "Message buffer convert error, pub_index= %ld", pub_index);
    return false;
  }
  std::shared_ptr<ROSMessageType> msg = msg_buffer->message_buffer.at(message_index);

  // Get publisher<ROSMessageType>
  std::shared_ptr<rclcpp::Publisher<ROSMessageType>> msg_publisher =
      std::dynamic_pointer_cast<rclcpp::Publisher<ROSMessageType>>(message_pubs_[pub_index]);
  if (!msg_publisher) {
    RCLCPP_ERROR(get_logger(), "Publisher convert error, pub_index= %ld", pub_index);
    return false;
  }

  // Publish msg
  if (header) {
    msg->header.stamp.sec = header->stamp.sec;
  }
  msg_publisher->publish(*msg);
  return true;
}

}  // namespace benchmark
}  // namespace qrb_ros

// Register QrbPlaybackNode as a component
RCLCPP_COMPONENTS_REGISTER_NODE(qrb_ros::benchmark::QrbPlaybackNode)