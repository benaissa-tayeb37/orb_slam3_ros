# ORB-SLAM3 ROS 2 Integration

## Overview
This repository provides a ROS 2 Humble integration for ORB-SLAM3, a state-of-the-art SLAM (Simultaneous Localization and Mapping) system. It is based on the original ROS 1 implementation from [thien94/orb_slam3_ros](https://github.com/thien94/orb_slam3_ros) and has been adapted for ROS 2 compatibility. Additionally, this repository has been optimized to work seamlessly with the RealSense D435 camera in both RGB-D and stereo modes.

## Features
- **ROS 2 Humble support**: Fully compatible with ROS 2 Humble.
- **Multi-camera support**: Monocular, stereo, and RGB-D cameras.
- **Optimized for RealSense D435**: Supports RGB-D and stereo modes for enhanced SLAM performance.
- **Real-time SLAM**: Efficient mapping and localization for robotics applications.
- **Loop closure detection**: Improves map accuracy and consistency.
- **Enhanced modularity**: Designed for ROS 2's node-based architecture.

## Installation
### Prerequisites
Ensure you have the following installed:
- ROS 2 Humble
- OpenCV
- Eigen
- Pangolin
- Intel RealSense SDK (`librealsense2`)

### Build Instructions
1. Clone the repository:
   ```bash
   git clone https://github.com/benaissa-tayeb37/orb_slam3_ros.git
   cd orb_slam3_ros
   ```

2. Build the package using `colcon`:
   ```bash
   colcon build
   ```

3. Source the workspace:
   ```bash
   source install/setup.bash
   ```

## Usage
### Launching ORB-SLAM3 with RealSense D435
Run the following command to start ORB-SLAM3 with RealSense D435 in RGB-D mode:
```bash
ros2 launch orb_slam3_ros rs_d435_rgbd_optimized.launch.py
```

For stereo mode, use:
```bash
ros2 launch orb_slam3_ros rs_d435_stereo_optimized.launch.py
```

### Configuration
Modify the `config.yaml` file to set camera parameters and SLAM options. Predefined configurations for RealSense D435 are included in the repository.

## Migration from ROS 1
This repository is adapted from the original ROS 1 implementation ([thien94/orb_slam3_ros](https://github.com/thien94/orb_slam3_ros)) and includes updates for ROS 2 Humble, such as:
- Conversion to ROS 2 nodes.
- Use of `colcon` for building.
- Updated launch files (`.launch.py` format).
- Optimizations for RealSense D435.

## Contributing
Contributions are welcome! Please fork the repository and submit a pull request.

## License
This project is licensed under the MIT License. See the `LICENSE` file for details.

## Contact
For questions or support, contact **tayeb.benaissa@g.enp.edu.dz**.