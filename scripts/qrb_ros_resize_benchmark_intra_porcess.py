"""
Performance test for qrb_ros_transport image

Required:
- Packages:
    - qrb_ros_transport
- Datasets:
    - /data/ros2_ws/qrb_ros_benchmark_data
"""

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from ros2_benchmark import ImageResolution
from ros2_benchmark import ROS2BenchmarkConfig, ROS2BenchmarkTest

IMAGE_RESOLUTION = ImageResolution.HD
ROSBAG_PATH = 'resize_data'

def launch_setup(container_prefix, container_sigterm_timeout):
    """Generate launch description for benchmarking XXX."""

    data_loader_node = ComposableNode(
        name='DataLoaderNode',
        namespace=TestResizeNode.generate_namespace(),
        package='ros2_benchmark',
        plugin='ros2_benchmark::DataLoaderNode',
        remappings=[('image_raw', '/data_loader_image')]
    )

    playback_node = ComposableNode(
        name='QrbPlaybackNode',
        namespace=TestResizeNode.generate_namespace(),
        package='qrb_ros_benchmark',
        plugin='qrb_ros::benchmark::QrbPlaybackNode',
        parameters=[{
            'data_formats': [
                'qrb_ros/transport/type/Image'
            ],
        }],
        remappings=[
            ('buffer/input0', '/data_loader_image'),
            ('input0', '/playback_image')
        ]
    )

    resize_node = ComposableNode(
        name='resize_1',
        namespace=TestResizeNode.generate_namespace(),
        package='qrb_ros_image_resize',
        plugin='qrb_ros::resize::ResizeNode',
        parameters=[{
            'calculate_enable':True,
            'use_scale': False,
            'height': 400,
            'width': 400,
            #'use_scale': True,
            #'height_scale': 0.25,
            #'width_scale': 0.25,
        }],
        remappings=[
            ('image_raw', '/playback_image'),
            ('image_resize', '/image_resize')
        ]
    )

    monitor_node = ComposableNode(
        name='QrbMonitorNode',
        namespace=TestResizeNode.generate_namespace(),
        package='qrb_ros_benchmark',
        plugin='qrb_ros::benchmark::QrbMonitorNode',
        parameters=[{
            'monitor_data_format': 'qrb_ros/transport/type/Image',
        }],
        remappings=[
            ('output', '/image_resize')
        ]
    )

    composable_node_container = ComposableNodeContainer(
        name='container',
        namespace=TestResizeNode.generate_namespace(),
        package='rclcpp_components',
        executable='component_container_mt',
        prefix=container_prefix,
        sigterm_timeout=container_sigterm_timeout,
        composable_node_descriptions=[
            data_loader_node,
            playback_node,
            resize_node,
            monitor_node,
        ],
        output='screen'
    )

    return [composable_node_container]

def generate_test_description():
    return TestResizeNode.generate_test_description_with_nsys(launch_setup)

class TestResizeNode(ROS2BenchmarkTest):
    """Performance test for XXX - qrb_ros/transport/type/Image."""

    # Custom configurations
    config = ROS2BenchmarkConfig(
        benchmark_name='qrb_ros_transport image test Node Benchmark',
        assets_root='/data/ros2_ws/',
        input_data_path=ROSBAG_PATH,
        # Upper and lower bounds of peak throughput search window
        publisher_upper_frequency=500.0,
        publisher_lower_frequency=10.0,
        # The number of frames to be buffered
        playback_message_buffer_size=20,
    )

    def test_benchmark(self):
        self.run_benchmark()