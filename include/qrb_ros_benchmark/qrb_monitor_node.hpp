// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_BENCHMARK__QRB_MONITOR_NODE_HPP_
#define QRB_ROS_BENCHMARK__QRB_MONITOR_NODE_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ros2_benchmark/monitor_node.hpp"
#include "qrb_ros_transport_image_type/image.hpp"
#include "qrb_ros_transport_imu_type/imu.hpp"
#include "qrb_ros_transport_point_cloud2_type/point_cloud2.hpp"
#include "dmabuf_transport/type/image.hpp"
#include "dmabuf_transport/type/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "qrb_ros_tensor_list_msgs/msg/tensor_list.hpp"

namespace qrb_ros
{
namespace benchmark
{

class QrbMonitorNode : public ros2_benchmark::MonitorNode
{
public:

  // QrbMonitorNode constructor.
  explicit QrbMonitorNode(const rclcpp::NodeOptions &);

private:

  // Determine the format, request to create the corresponding type of subscriber.
  void create_monitor_subscriber();

  // The method used to create the ROSMessageType subscriber.
  template<typename ROSMessageType>
  void create_message_subscriber();

  // Message subscriber callback function.
  template<typename ROSMessageType>
  void ros_message_subscriber_callback(const std::shared_ptr<ROSMessageType> ros_msg);
};

}  // namespace benchmark
}  // namespace qrb_ros

#endif  // QRB_ROS_BENCHMARK__QRB_MONITOR_NODE_HPP_
