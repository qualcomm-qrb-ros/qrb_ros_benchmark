// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_BENCHMARK__QRB_PLAYBACK_NODE_HPP_
#define QRB_ROS_BENCHMARK__QRB_PLAYBACK_NODE_HPP_

// Standard library
#include <string>
#include <unordered_map>
#include <vector>

// ROS libraries
#include "rclcpp/rclcpp.hpp"
#include "ros2_benchmark/playback_node.hpp"

// QRB message types (includes all necessary message type headers)
#include "qrb_ros_benchmark/qrb_message_types.hpp"

namespace qrb_ros
{
namespace benchmark
{

enum class PubSubType : uint8_t
{
  QrbTransportType = 0,
  DmabufTransportType,
  RosMessaageType,
  GenericType,
};

class MessageBufferBase {
public:
  std::string message_format;
  virtual size_t get_buffer_size();
  virtual void clear();
};

template<typename ROSMessageType>
class MessageBuffer : public qrb_ros::benchmark::MessageBufferBase
{
public:
  std::vector<std::shared_ptr<ROSMessageType>> message_buffer;
  size_t get_buffer_size() override;
  void clear() override;
};

class QrbPlaybackNode : public ros2_benchmark::PlaybackNode
{
public:
  // QrbPlaybackNode constructor.
  explicit QrbPlaybackNode(const rclcpp::NodeOptions &);

private:
  // Check if the messages buffer is full for request size.
  bool AreBuffersFull() const override;

  // Clear message buffers.
  void ClearBuffers() override;

  // Use the selected publisher to publish message, and the timestamp
  // will be modified before publishing.
  bool PublishMessage(
    const size_t pub_index,
    const size_t message_index,
    const std::optional<std_msgs::msg::Header> & header) override;

  // Return the messages count number.
  uint64_t GetRecordedMessageCount() const override;

  // Return the messages count number for the specified pub.
  uint64_t GetRecordedMessageCount(size_t pub_index) const override;

  // Return publishers count number.
  size_t GetPublisherCount() const override;

  // Determine the format, request to create the corresponding type of pub and sub.
  void create_playback_pubsub(const size_t index, const std::string format);

  // Create publisher and subscriber with ROSMessageType.
  template<typename ROSMessageType>
  void create_message_pubsub(const size_t index, const std::string format);

  // Message subscriber callback function, which is used to record the received messages.
  template<typename ROSMessageType>
  void message_subscriber_callback(const std::shared_ptr<ROSMessageType> msg, const size_t index);

  // Use the selected publisher<ROSMessageType> to publish message.
  template<typename ROSMessageType>
  bool publish_message(
    const size_t pub_index,
    const size_t message_index,
    const std::optional<std_msgs::msg::Header> & header);

  // A map to record the pub/sub types and indices, map key is pub/sub index.
  std::unordered_map<size_t, PubSubType> pub_sub_types_;

  // A map to record the publisher and indices, map key is pub/sub index.
  std::unordered_map<size_t, std::shared_ptr<rclcpp::PublisherBase>> message_pubs_;

  // A map to record the MessageBuffer<ROSMessageType> and indices, map key is pub/sub index.
  std::unordered_map<size_t, std::shared_ptr<MessageBufferBase>> message_buffer_map_;
};

}  // namespace benchmark
}  // namespace qrb_ros

#endif  // QRB_ROS_BENCHMARK__QRB_PLAYBACK_NODE_HPP_
