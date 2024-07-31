# Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear

"""
Live benchmarking qrb_ros_imu node.

BenchmarkMode: BenchmarkMode.LIVE

Pipeline under Test:
    1. qrb_ros::imu::ImuComponent
    2. qrb_ros::benchmark::QrbMonitorNode

Dependency package:
    1. qrb_ros_imu
    2. qrb_ros_benchmark
"""

import os

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from ros2_benchmark import ImageResolution
from ros2_benchmark import MonitorPerformanceCalculatorsInfo
from ros2_benchmark import ROS2BenchmarkConfig, ROS2BenchmarkTest
from ros2_benchmark import BasicPerformanceCalculator, BenchmarkMode

IMAGE_RESOLUTION = ImageResolution.VGA

def launch_setup(container_prefix, container_sigterm_timeout):
    """qrb_ros_benchmark live benchmarking for qrb_ros_imu."""

    qrb_ros_imu_node = ComposableNode(
        name='QrbRosIMU',
        namespace=TestQrbRosImuNode.generate_namespace(),
        package='qrb_ros_imu',
        plugin='qrb_ros::imu::ImuComponent',
        remappings=[('imu', '/imu_raw')]
    )

    qrb_ros_monitor_node = ComposableNode(
        name='QrbMonitorNode',
        namespace=TestQrbRosImuNode.generate_namespace(),
        package='qrb_ros_benchmark',
        plugin='qrb_ros::benchmark::QrbMonitorNode',
        parameters=[{
            'monitor_index': 1,
            'monitor_data_format': 'qrb_ros/transport/type/Imu',
        }],
        remappings=[('output', '/imu_raw')]
    )

    composable_node_container = ComposableNodeContainer(
        name='composable_node_container',
        namespace=TestQrbRosImuNode.generate_namespace(),
        package='rclcpp_components',
        executable='component_container_mt',
        sigterm_timeout=container_sigterm_timeout,
        prefix=container_prefix,
        composable_node_descriptions=[
            qrb_ros_imu_node,
            qrb_ros_monitor_node,
        ],
        output='screen'
    )

    return [composable_node_container]

def generate_test_description():
    return TestQrbRosImuNode.generate_test_description_with_nsys(launch_setup)


class TestQrbRosImuNode(ROS2BenchmarkTest):
    """Qrb Ros Benchamrk Live performance test for Qrb Ros IMU."""

    # Custom configuration parameters
    config = ROS2BenchmarkConfig(
        benchmark_name='Qrb Ros Imu Live Benchmark',
        # BenchmarkMode.LIVE is used to test datasource case.
        # Can be either of "TIMELINE", "LOOPING" or "SWEEPING".
        benchmark_mode=BenchmarkMode.LIVE,
        # How long each benchmark test iteration should run, the unit is seconds.
        benchmark_duration=5,
        # The number of benchmark test iterations.
        test_iterations=5,
        # record topic msg timestamps as start time
        collect_start_timestamps_from_monitors=True,
        # Custom information will be included in the report file.
        custom_report_info={'data_resolution': IMAGE_RESOLUTION},
        # Monitor node configurations
        monitor_info_list=[
            MonitorPerformanceCalculatorsInfo(
                'monitor_node1',
                [BasicPerformanceCalculator({
                    'report_prefix': 'qrb_ros_imu',
                    'message_key_match': True
                })])
        ]
    )

    def test_benchmark(self):
        self.run_benchmark()