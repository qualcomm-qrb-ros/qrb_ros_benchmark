// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_BENCHMARK__QRB_MESSAGE_TYPES_HPP_
#define QRB_ROS_BENCHMARK__QRB_MESSAGE_TYPES_HPP_

// Standard library
#include <string>
#include <vector>

// QRB ROS transport types
#include "qrb_ros_transport_image_type/image.hpp"
#include "qrb_ros_transport_imu_type/imu.hpp"
#include "qrb_ros_transport_point_cloud2_type/point_cloud2.hpp"

// DMABUF transport types
#include "dmabuf_transport/type/image.hpp"
#include "dmabuf_transport/type/point_cloud2.hpp"

// ROS message types
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "qrb_ros_tensor_list_msgs/msg/tensor_list.hpp"

namespace qrb_ros
{
namespace benchmark
{

// Common macro for creating ROS message subscriber
#define FOR_EACH_ROS_MESSAGE_TYPE(MACRO) \
  MACRO(sensor_msgs::msg::Image) \
  MACRO(sensor_msgs::msg::CompressedImage) \
  MACRO(qrb_ros_tensor_list_msgs::msg::TensorList)

// Common macro for creating QRB transport type subscriber/publisher
#define FOR_EACH_QRB_TRANSPORT_TYPE(MACRO) \
  MACRO("qrb_ros/transport/type/Image", qrb_ros::transport::type::Image) \
  MACRO("qrb_ros/transport/type/Imu", qrb_ros::transport::type::Imu)

// Common macro for creating dmabuf transport type subscriber/publisher
#define FOR_EACH_DMABUF_TRANSPORT_TYPE(MACRO) \
  MACRO("dmabuf_transport/type/Image", dmabuf_transport::type::Image) \
  MACRO("dmabuf_transport/type/PointCloud2", dmabuf_transport::type::PointCloud2)

}  // namespace benchmark
}  // namespace qrb_ros

#endif  // QRB_ROS_BENCHMARK__QRB_MESSAGE_TYPES_HPP_