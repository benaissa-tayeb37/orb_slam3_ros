# ORB-SLAM3 ROS 2 Integration

## Overview

This repository provides a ROS 2 Humble integration for ORB-SLAM3, a state-of-the-art SLAM (Simultaneous Localization and Mapping) system. It is based on the original ROS 1 implementation from [thien94/orb_slam3_ros](https://github.com/thien94/orb_slam3_ros) and has been adapted for ROS 2 compatibility. Additionally, this repository has been optimized to work seamlessly with the RealSense D435 camera in both RGB-D and stereo modes.

### Key Features

- **ROS 2 Humble Support**: Fully compatible with ROS 2 Humble distribution
- **Multi-Camera Support**: Compatible with monocular, stereo, and RGB-D cameras
- **RealSense D435 Optimization**: Enhanced support for RGB-D and stereo modes
- **Real-Time SLAM**: Efficient mapping and localization for robotics applications
- **Loop Closure Detection**: Improves map accuracy and consistency over time
- **Enhanced Modularity**: Designed specifically for ROS 2's node-based architecture

---

## Table of Contents

- [Installation](#installation)
- [Usage](#usage)
- [Configuration](#configuration)
- [Visualization](#visualization)
- [Migration from ROS 1](#migration-from-ros-1)
- [File Structure](#file-structure)
- [Supported Cameras](#supported-cameras)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [License](#license)
- [Contact](#contact)
- [Acknowledgments](#acknowledgments)

---

## Installation

### Prerequisites

Ensure you have the following dependencies installed on your system:

- **ROS 2 Humble**: [Installation Guide](https://docs.ros.org/en/humble/Installation.html)
- **OpenCV** (>=4.5): Computer vision library
- **Eigen3**: Linear algebra library
- **Pangolin**: 3D visualization library
- **Boost**: C++ libraries
- **Intel RealSense SDK** (`librealsense2`): For RealSense camera support

#### Installing Dependencies on Ubuntu 22.04

```bash
# Update package list
sudo apt update

# Install ROS 2 Humble (if not already installed)
# Follow the official ROS 2 Humble installation guide

# Install system dependencies
sudo apt install -y \
    libopencv-dev \
    libeigen3-dev \
    libboost-all-dev \
    libssl-dev \
    libusb-1.0-0-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev

# Install Pangolin
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build && cd build
cmake ..
make -j4
sudo make install

# Install Intel RealSense SDK
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
sudo apt install -y librealsense2-dkms librealsense2-utils librealsense2-dev librealsense2-dbg
```

### Build Instructions

1. **Create a ROS 2 workspace** (if you don't have one):
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```

2. **Clone the repository**:
   ```bash
   git clone https://github.com/benaissa-tayeb37/orb_slam3_ros.git
   cd ~/ros2_ws
   ```

3. **Install ROS 2 dependencies**:
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```

4. **Build the package using colcon**:
   ```bash
   colcon build --packages-select orb_slam3_ros
   ```

5. **Source the workspace**:
   ```bash
   source install/setup.bash
   ```

---

## Usage

### Launching ORB-SLAM3

The repository provides several launch files for different SLAM modes:

#### Monocular SLAM
For single camera SLAM using a monocular camera:
```bash
ros2 launch orb_slam3_ros mono.launch.py
```

#### Stereo SLAM
For stereo SLAM using RealSense D435 in stereo mode:
```bash
ros2 launch orb_slam3_ros rs_d435_stereo_optimized.launch.py
```

#### RGB-D SLAM
For RGB-D SLAM using RealSense D435 with depth information:
```bash
ros2 launch orb_slam3_ros rs_d435_rgbd_optimized.launch.py
```

### Launch Parameters

You can customize the launch behavior using parameters:

```bash
# Example with custom vocabulary file
ros2 launch orb_slam3_ros rs_d435_rgbd_optimized.launch.py \
    voc_file:=/path/to/your/vocabulary.txt \
    settings_file:=/path/to/your/config.yaml
```

---

## Configuration

### Camera Calibration

Camera parameters are configured through YAML files located in the `config/` directory. For RealSense D435, predefined configurations are provided:

- `rs_d435_rgbd.yaml`: RGB-D mode configuration
- `rs_d435_stereo.yaml`: Stereo mode configuration

### Custom Camera Configuration

To use a custom camera, create a new YAML configuration file with the following structure:

```yaml
Camera:
  name: "Custom Camera"
  setup: "monocular"  # monocular, stereo, or rgbd
  
  # Camera resolution
  width: 640
  height: 480
  fps: 30.0
  
  # Camera matrix parameters
  fx: 525.0
  fy: 525.0
  cx: 319.5
  cy: 239.5
  
  # Distortion parameters
  k1: 0.0
  k2: 0.0
  p1: 0.0
  p2: 0.0
  k3: 0.0

# ORB Parameters
ORBextractor:
  nFeatures: 1000
  scaleFactor: 1.2
  nLevels: 8
  iniThFAST: 20
  minThFAST: 7

# Viewer Parameters
Viewer:
  KeyFrameSize: 0.05
  KeyFrameLineWidth: 1
  GraphLineWidth: 0.9
  PointSize: 2
  CameraSize: 0.08
  CameraLineWidth: 3
  ViewpointX: 0
  ViewpointY: -0.7
  ViewpointZ: -1.8
  ViewpointF: 500
```

---

## Visualization

### RViz Configuration

RViz configuration files are provided for visualization:

- `config/orb_slam3_rgbd.rviz`: Configuration for RGB-D SLAM
- `config/orb_slam3_stereo.rviz`: Configuration for stereo SLAM

To launch RViz with the appropriate configuration:

```bash
# For RGB-D visualization
rviz2 -d ~/ros2_ws/src/orb_slam3_ros/config/orb_slam3_rgbd.rviz

# For stereo visualization
rviz2 -d ~/ros2_ws/src/orb_slam3_ros/config/orb_slam3_stereo.rviz
```

### Published Topics

The ORB-SLAM3 node publishes the following topics:

- `/orb_slam3/camera_pose`: Current camera pose
- `/orb_slam3/map_points`: 3D map points
- `/orb_slam3/keyframe_trajectory`: Keyframe trajectory
- `/orb_slam3/tracking_image`: Image with tracking features

### Subscribed Topics

The node subscribes to:

- `/camera/image_raw`: Camera image (monocular/stereo)
- `/camera/depth/image_raw`: Depth image (RGB-D mode)
- `/camera/camera_info`: Camera information

---

## Migration from ROS 1

This repository is adapted from the original ROS 1 implementation ([thien94/orb_slam3_ros](https://github.com/thien94/orb_slam3_ros)) with the following key changes:

### Major Updates for ROS 2

- **Node Architecture**: Converted to ROS 2 node structure using `rclcpp`
- **Build System**: Migrated from `catkin_make` to `colcon` build system
- **Launch Files**: Updated to Python-based launch files (`.launch.py` format)
- **Message Types**: Updated to ROS 2 message types and interfaces
- **Parameter System**: Adapted to ROS 2 parameter declaration and handling
- **RealSense Integration**: Enhanced support and optimization for RealSense D435

### Compatibility Notes

- All original ORB-SLAM3 algorithms and features are preserved
- Performance optimizations specific to ROS 2 Humble
- Improved memory management and thread safety

---

## File Structure

```
orb_slam3_ros/
├── src/                          # ROS 2 source files
│   ├── monocular/               # Monocular SLAM node
│   ├── stereo/                  # Stereo SLAM node
│   └── rgbd/                    # RGB-D SLAM node
├── launch/                      # Launch files (.launch.py)
│   ├── mono.launch.py
│   ├── rs_d435_stereo_optimized.launch.py
│   └── rs_d435_rgbd_optimized.launch.py
├── config/                      # Configuration files
│   ├── orb_slam3_rgbd.rviz     # RViz configuration for RGB-D
│   ├── orb_slam3_stereo.rviz   # RViz configuration for stereo
│   ├── rs_d435_rgbd.yaml       # RealSense D435 RGB-D settings
│   └── rs_d435_stereo.yaml     # RealSense D435 stereo settings
├── orb_slam3/                   # ORB-SLAM3 core library
│   ├── include/                 # Header files
│   ├── src/                     # Source files
│   └── Thirdparty/             # Third-party dependencies
├── Vocabulary/                  # ORB vocabulary files
│   └── ORBvoc.txt              # Pre-trained vocabulary
├── CMakeLists.txt              # CMake build configuration
├── package.xml                 # ROS 2 package manifest
└── README.md                   # This file
```

---

## Supported Cameras

### Tested Cameras

- **RGB-D Cameras**:
  - Intel RealSense D435 ✅
  - Intel RealSense D455 ✅
  - Microsoft Kinect v1/v2 ⚠️ (requires additional drivers)

- **Stereo Cameras**:
  - Intel RealSense D435 (stereo mode) ✅
  - ZED Camera ⚠️ (requires ZED SDK)
  - Custom stereo rigs ⚠️ (requires calibration)

- **Monocular Cameras**:
  - USB webcams ✅
  - Laptop built-in cameras ✅
  - Industrial cameras ⚠️ (requires proper drivers)

### Camera Requirements

- **Minimum Resolution**: 640x480
- **Recommended FPS**: 30 FPS or higher
- **Calibration**: Proper camera calibration is essential for good SLAM performance

---

## Troubleshooting

### Common Issues and Solutions

#### 1. Camera Not Detected

**Symptoms**: No image topics published, camera initialization fails

**Solutions**:
```bash
# Check if camera is detected
lsusb | grep Intel  # For RealSense cameras

# Test RealSense camera
realsense-viewer    # Should show camera feed

# Check ROS 2 camera topics
ros2 topic list | grep camera
```

#### 2. SLAM Initialization Fails

**Symptoms**: No map points generated, tracking fails immediately

**Solutions**:
- Verify camera calibration parameters in config file
- Ensure sufficient texture and lighting in the environment
- Check that the vocabulary file path is correct
- Verify image topics are being published:
  ```bash
  ros2 topic echo /camera/image_raw --max-count 1
  ```

#### 3. Poor Tracking Performance

**Symptoms**: Frequent tracking loss, inaccurate poses

**Solutions**:
- Improve lighting conditions
- Reduce camera motion speed
- Adjust ORB feature parameters in config file
- Ensure camera is properly calibrated

#### 4. RViz Visualization Issues

**Symptoms**: Empty RViz display, missing visualizations

**Solutions**:
```bash
# Check if topics are being published
ros2 topic list | grep orb_slam3

# Verify coordinate frames
ros2 run tf2_tools view_frames.py

# Launch RViz with correct config
rviz2 -d /path/to/correct/config.rviz
```

#### 5. Build Errors

**Symptoms**: Compilation fails, missing dependencies

**Solutions**:
```bash
# Clean build directory
rm -rf build/ install/ log/

# Install missing dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build with verbose output
colcon build --packages-select orb_slam3_ros --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Performance Optimization

For better performance:

1. **CPU Optimization**:
   ```bash
   # Build with optimizations
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

2. **Memory Usage**:
   - Reduce `nFeatures` in ORB parameters for lower memory usage
   - Adjust `scaleFactor` and `nLevels` based on your environment

3. **Real-time Performance**:
   - Ensure adequate CPU resources
   - Consider using dedicated GPU for other processes
   - Monitor system resources: `htop`, `nvidia-smi`

---

## Contributing

We welcome contributions to improve this ROS 2 integration! Here's how you can help:

### How to Contribute

1. **Fork the repository**
2. **Create a feature branch**:
   ```bash
   git checkout -b feature/your-feature-name
   ```
3. **Make your changes**
4. **Test thoroughly**
5. **Submit a pull request**

### Contribution Guidelines

- Follow existing code style and conventions
- Add appropriate documentation for new features
- Include test cases where applicable
- Update README.md if necessary
- Ensure compatibility with ROS 2 Humble

### Areas for Contribution

- Support for additional camera models
- Performance optimizations
- Enhanced visualization features
- Documentation improvements
- Bug fixes and stability improvements

---

## License

This project is licensed under the **MIT License**. See the [LICENSE](LICENSE) file for full details.

### Third-Party Licenses

- ORB-SLAM3: [GPLv3 License](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/License-gpl.txt)
- Pangolin: [MIT License](https://github.com/stevenlovegrove/Pangolin/blob/master/LICENSE)

---

## Contact

For questions, support, or collaboration opportunities, please contact:

**Tayeb Benaissa**  
📧 Email: tayeb.benaissa@g.enp.edu.dz  
🐙 GitHub: [@benaissa-tayeb37](https://github.com/benaissa-tayeb37)

### Reporting Issues

Please use the [GitHub Issues](https://github.com/benaissa-tayeb37/orb_slam3_ros/issues) page to report bugs or request features.

---

## Acknowledgments

This project builds upon the excellent work of several contributors:

### Original ORB-SLAM3 Authors
- **Carlos Campos** - University of Zaragoza
- **Richard Elvira** - University of Zaragoza  
- **Juan J. Gómez Rodríguez** - University of Zaragoza
- **José M.M. Montiel** - University of Zaragoza
- **Juan D. Tardós** - University of Zaragoza

### ROS Integration
- **thien94** - Original ROS 1 integration ([orb_slam3_ros](https://github.com/thien94/orb_slam3_ros))

### References

If you use this work in your research, please cite:

```bibtex
@article{campos2021orb,
  title={ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual–Inertial, and Multimap SLAM},
  author={Campos, Carlos and Elvira, Richard and Rodríguez, Juan J Gómez and Montiel, José MM and Tardós, Juan D},
  journal={IEEE Transactions on Robotics},
  volume={37},
  number={6},
  pages={1874--1890},
  year={2021},
  publisher={IEEE}
}
```

---

## Changelog

### Version 2.0.0 (ROS 2 Humble)
- ✅ Complete migration to ROS 2 Humble
- ✅ Enhanced RealSense D435 support
- ✅ Improved visualization with RViz2
- ✅ Updated build system to colcon
- ✅ Python-based launch files

### Version 1.0.0 (ROS 1)
- ✅ Initial ROS 1 implementation
- ✅ Monocular, stereo, and RGB-D support
- ✅ Basic RealSense integration

---

*Last updated: June 2025*