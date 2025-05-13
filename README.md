# QRB ROS Benchmark

## Overview

[QRB ROS Benchmark](https://github.com/qualcomm-qrb-ros/qrb_ros_benchmark) is a benchmarking tool designed for evaluating performance of ROS components on Qualcomm robotics platforms. It provides reusable components for benchmarking various message types and ROS nodes, with a focus on zero-copy transport mechanisms.

This package builds on [ros2_benchmark](https://github.com/ros-acceleration/ros2_benchmark) and extends it with specialized components for benchmarking QRB ROS transport types, including:

- QRB transport types (Image, IMU, PointCloud2)
- DMABuf transport types (Image, PointCloud2)
- Standard ROS message types

## Getting Started

### Build

For the Qualcomm QCLinux platform, we provide two ways to build this package:

<details>
<summary>On-Device Compilation with Docker</summary>

1. Set up the QCLinux Docker environment following the [QRB ROS Docker Setup](https://github.com/qualcomm-qrb-ros/qrb_ros_docker?tab=readme-ov-file#quickstart).

2. Clone and build the source code:

    ```bash
    cd ~/qrb_ros_ws/src/qrb_ros_docker/scripts
    bash docker_run.sh

    # Clone this repository and dependencies
    git clone https://github.com/qualcomm-qrb-ros/ros2_benchmark.git
    git clone https://github.com/qualcomm-qrb-ros/qrb_ros_transport.git
    git clone https://github.com/qualcomm-qrb-ros/qrb_ros_benchmark.git
    git clone https://github.com/qualcomm-qrb-ros/lib_mem_dmabuf.git
    git clone https://github.com/qualcomm-qrb-ros/qrb_ros_imu.git
    git clone https://github.com/qualcomm-qrb-ros/dmabuf_transport.git

    # Build packages
    colcon build --packages-select qrb_ros_benchmark
    ```

</details>

<details>
<summary>Cross Compilation with QIRP SDK</summary>

1. Set up the QIRP SDK environment: Refer to [QRB ROS Documents: Getting Started](https://qualcomm-qrb-ros.github.io/main/getting_started/environment_setup.html).

2. Create a workspace and clone the source code:

    ```bash
    mkdir -p <qirp_decompressed_workspace>/qirp-sdk/ros_ws
    cd <qirp_decompressed_workspace>/qirp-sdk/ros_ws

    git clone https://github.com/qualcomm-qrb-ros/ros2_benchmark.git
    git clone https://github.com/qualcomm-qrb-ros/qrb_ros_transport.git
    git clone https://github.com/qualcomm-qrb-ros/qrb_ros_benchmark.git
    git clone https://github.com/qualcomm-qrb-ros/lib_mem_dmabuf.git
    git clone https://github.com/qualcomm-qrb-ros/qrb_ros_imu.git
    git clone https://github.com/qualcomm-qrb-ros/dmabuf_transport.git
    ```

3. Build the source code with QIRP SDK:

    ```bash
    colcon build --merge-install --packages-select qrb_ros_benchmark --cmake-args \
      -DPYTHON_EXECUTABLE=${OECORE_NATIVE_SYSROOT}/usr/bin/python3 \
      -DPython3_NumPy_INCLUDE_DIR=${OECORE_NATIVE_SYSROOT}/usr/lib/python3.12/site-packages/numpy/core/include \
      -DPYTHON_SOABI=cpython-312-aarch64-linux-gnu \
      -DCMAKE_MAKE_PROGRAM=/usr/bin/make \
      -DBUILD_TESTING=OFF
    ```

</details>

### Usage

This package provides two main components:

1. `QrbPlaybackNode`: Manages message playback from buffers with various message types
2. `QrbMonitorNode`: Records timing information for performance benchmarking

#### Creating a benchmark script

Here's an example of how to use these components in a benchmark script:

```python
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

from ros2_benchmark import ROS2BenchmarkConfig, ROS2BenchmarkTest
from ros2_benchmark import MonitorPerformanceCalculatorsInfo
from ros2_benchmark import BasicPerformanceCalculator, BenchmarkMode

def launch_setup(container_prefix, container_sigterm_timeout):
    """Generate launch description for benchmarking a node."""

    # Configure the playback node
    playback_node = ComposableNode(
        name='QrbPlaybackNode',
        namespace=TestBenchmarkNode.generate_namespace(),
        package='qrb_ros_benchmark',
        plugin='qrb_ros::benchmark::QrbPlaybackNode',
        parameters=[{
            'data_formats': [
                'qrb_ros/transport/type/Image'
            ],
        }],
        remappings=[
            ('buffer/input0', '/input_topic'),
            ('input0', '/output_topic')
        ]
    )

    # Configure the node to benchmark
    node_under_test = ComposableNode(
        name='NodeUnderTest',
        namespace=TestBenchmarkNode.generate_namespace(),
        package='your_package',
        plugin='your_namespace::YourNodePlugin',
        parameters=[{
            # Your node parameters
        }],
        remappings=[
            ('input', '/output_topic'),
            ('output', '/result_topic')
        ]
    )

    # Configure the monitor node
    monitor_node = ComposableNode(
        name='QrbMonitorNode',
        namespace=TestBenchmarkNode.generate_namespace(),
        package='qrb_ros_benchmark',
        plugin='qrb_ros::benchmark::QrbMonitorNode',
        parameters=[{
            'monitor_data_format': 'qrb_ros/transport/type/Image',
        }],
        remappings=[
            ('output', '/result_topic')
        ]
    )

    composable_node_container = ComposableNodeContainer(
        name='container',
        namespace=TestBenchmarkNode.generate_namespace(),
        package='rclcpp_components',
        executable='component_container_mt',
        prefix=container_prefix,
        sigterm_timeout=container_sigterm_timeout,
        composable_node_descriptions=[
            playback_node,
            node_under_test,
            monitor_node,
        ],
        output='screen'
    )

    return [composable_node_container]

def generate_test_description():
    return TestBenchmarkNode.generate_test_description_with_nsys(launch_setup)

class TestBenchmarkNode(ROS2BenchmarkTest):
    """Performance benchmark test for your node."""

    # Custom configurations
    config = ROS2BenchmarkConfig(
        benchmark_name='Your Node Benchmark',
        benchmark_duration=5,
        test_iterations=5,
        # Other configuration parameters as needed
    )

    def test_benchmark(self):
        self.run_benchmark()
```

For more complete examples, check the included benchmark scripts in the `/scripts` directory.

## Supported Message Types

The benchmark framework supports the following message types:

### QRB Transport Types
- `qrb_ros/transport/type/Image`
- `qrb_ros/transport/type/Imu`
- `qrb_ros/transport/type/PointCloud2`

### DMABuf Transport Types
- `dmabuf_transport/type/Image` 
- `dmabuf_transport/type/PointCloud2`

### ROS Message Types
- `sensor_msgs::msg::Image`
- `sensor_msgs::msg::CompressedImage`
- `qrb_ros_tensor_list_msgs::msg::TensorList`

## Benchmark Results

Benchmark results are generated in JSON format and stored in the `results` directory. These files include performance metrics such as:

- Frame rates (mean playback and processing)
- Latency statistics (min, max, mean)
- Jitter statistics (min, max, mean, std dev)
- CPU utilization metrics

## Contributing

We welcome contributions to QRB ROS Benchmark! Whether they are features, issues, documentation, guides, or anything else, please refer to our [contribution guidelines](./CONTRIBUTING.md) and [code of conduct](./CODE-OF-CONDUCT.md).

- Bug report: If you see an error message or encounter failures, please create a [bug report](../../issues)
- Feature Request: If you have an idea for an improvement, please submit a [feature request](../../issues)

## Authors

* **Zhaoyuan Cheng** - *Maintainer* - [@quic-zhaoyuan](https://github.com/quic-zhaoyuan)

See also the list of [contributors](https://github.com/qualcomm-qrb-ros/qrb_ros_benchmark/contributors) who participated in this project.

## License

Project is licensed under the [BSD-3-Clause License](https://spdx.org/licenses/BSD-3-Clause.html). See [LICENSE](./LICENSE) for the full license text.