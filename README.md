# QRB ROS Benchmark

qrb_ros_benchmark is a ros package that extends the functionality of opensource [ROS2 Benchmark](https://github.com/NVIDIA-ISAAC-ROS/ros2_benchmark) package.

## Overview

qrb_ros_benchmark is a ros package that extends the functionality of opensource ros2_benchmark package. qrb_ros_benchmark supports testing the performance of ROS nodes after accelerating by the dmabuf_transport / qrb_ros_transport function. [QRB ROS Transport](https://github.com/qualcomm-qrb-ros/qrb_ros_transport), it leverages type adaption and intra process communication to optimize message formats and dramatically accelerate communication between participating nodes.


## Build

Currently, we only support use QCLINUX to build

1. Setup environments follow this document 's [Set up the cross-compile environment.](https://docs.qualcomm.com/bundle/publicresource/topics/80-65220-2/develop-your-first-application_6.html?product=1601111740013072&facet=Qualcomm%20Intelligent%20Robotics%20(QIRP)%20Product%20SDK&state=releasecandidate) part

2. Create `ros_ws` directory in `<qirp_decompressed_workspace>/qirp-sdk/`

3. Clone this repository under `<qirp_decompressed_workspace>/qirp-sdk/ros_ws`
     ```bash
     git clone https://github.com/qualcomm-qrb-ros/lib_mem_dmabuf.git
     git clone https://github.com/qualcomm-qrb-ros/qrb_ros_imu.git
     git clone https://github.com/qualcomm-qrb-ros/qrb_ros_transport.git
     git clone https://github.com/qualcomm-qrb-ros/dmabuf_transport.git
     git clone https://github.com/qualcomm-qrb-ros/qrb_ros_tensor_list_msgs.git
     git clone https://github.com/NVIDIA-ISAAC-ROS/ros2_benchmark.git
     git clone https://github.com/qualcomm-qrb-ros/qrb_ros_benchmark.git
     ```

4. Add dependencies in ros2_benhcmark/ros2_benchmark/CMakeLists.txt
     ```bash
     find_package(rosbag2_compression REQUIRED)
     ```

5. Modify line 58 of the `/qirp-sdk/toolchain/install_dir/sysroots/armv8-2a-qcom-linux/usr/share/rosbag2_compression_zstd/cmake/export_rosbag2_compression_zstdExport.cmake` file to:
     ```bash
     INTERFACE_LINK_LIBRARIES "rcpputils::rcpputils;rosbag2_compression::rosbag2_compression;${_IMPORT_PREFIX}/lib/libzstd.so"
     ```

6. Build this project
     ```bash
     export AMENT_PREFIX_PATH="${OECORE_TARGET_SYSROOT}/usr;${OECORE_NATIVE_SYSROOT}/usr"
     export PYTHONPATH=${PYTHONPATH}:${OECORE_TARGET_SYSROOT}/usr/lib/python3.10/site-packages

     colcon build --merge-install --cmake-args \
       -DPython3_ROOT_DIR=${OECORE_TARGET_SYSROOT}/usr \
       -DPython3_NumPy_INCLUDE_DIR=${OECORE_TARGET_SYSROOT}/usr/lib/python3.10/site-packages/numpy/core/include \
       -DPYTHON_SOABI=cpython-310-aarch64-linux-gnu -DCMAKE_STAGING_PREFIX=$(pwd)/install \
       -DCMAKE_PREFIX_PATH=$(pwd)/install/share \
       -DBUILD_TESTING=OFF
     ```

7. Push to the device & Install
     ```bash
     cd `<qirp_decompressed_workspace>/qirp-sdk/ros_ws/install`
     tar czvf qrb_ros_benchmark.tar.gz lib share
     scp qrb_ros_benchmark.tar.gz root@[ip-addr]:/opt/
     ssh root@[ip-addr]
     (ssh) tar -zxf /opt/qrb_ros_benchmark.tar.gz -C /opt/qcom/qirp-sdk/usr/
     ```

## Run

This package supports running it directly from the command with test script.

a.Run with command

1. Source this file to set up the environment on your device:
    ```bash
    ssh root@[ip-addr]
    (ssh) export HOME=/opt
    (ssh) source /opt/qcom/qirp-sdk/qirp-setup.sh
    (ssh) export ROS_DOMAIN_ID=xx
    (ssh) source /usr/bin/ros_setup.bash
    ```

2. Use this command to run this package
    ```bash
    (ssh) launch_test xx/qrb_ros_imu_benchmark.py
    ```


## Acceleration Performance Test

This package is based on ROS2 Benchmark, and additional support to test the  [QRB ROS Transport](https://github.com/qualcomm-qrb-ros/qrb_ros_transport) optimize message formats and accelerate communication between participating nodes.

## Packages

Will update in the future.


## Contributions

Thanks for your interest in contributing to qrb_ros_benchmark! Please read our [Contributions Page](CONTRIBUTING.md) for more information on contributing features or bug fixes. We look forward to your participation!

## License

qrb_ros_benchmark is licensed under the BSD-3-clause "New" or "Revised" License.

Check out the [LICENSE](LICENSE) for more details.
