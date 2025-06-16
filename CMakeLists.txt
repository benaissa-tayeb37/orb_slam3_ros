cmake_minimum_required(VERSION 3.5)
project(orb_slam3_ros)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

message("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rmw REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(rmw_fastrtps_cpp REQUIRED)
# OpenCV
find_package(OpenCV 4.2 REQUIRED)
if(NOT OpenCV_FOUND)
  message(FATAL_ERROR "OpenCV 4.2 not found.")
endif()
message("OPENCV VERSION: ${OpenCV_VERSION}")

# Other dependencies
find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

# Generate services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SaveMap.srv"
  DEPENDENCIES std_msgs
)

# Setup include directories
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/orb_slam3
  ${PROJECT_SOURCE_DIR}/orb_slam3/include
  ${PROJECT_SOURCE_DIR}/orb_slam3/include/CameraModels
  ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty
  ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/Sophus
  ${EIGEN3_INCLUDE_DIR}
  ${Pangolin_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${rmw_INCLUDE_DIRS}
  ${rmw_implementation_INCLUDE_DIRS} 
  ${rmw_fastrtps_cpp_INCLUDE_DIRS}
  /opt/ros/humble/include
  /opt/ros/humble/include/rmw
)

# Include third-party libraries
include(${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/DBoW2/CMakeLists.txt)
include(${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/g2o/CMakeLists.txt)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/orb_slam3/lib)

# Build ORB-SLAM3 library
add_library(${PROJECT_NAME}_core SHARED
  orb_slam3/src/System.cc
  orb_slam3/src/Tracking.cc
  orb_slam3/src/LocalMapping.cc
  orb_slam3/src/LoopClosing.cc
  orb_slam3/src/ORBextractor.cc
  orb_slam3/src/ORBmatcher.cc
  orb_slam3/src/FrameDrawer.cc
  orb_slam3/src/Converter.cc
  orb_slam3/src/MapPoint.cc
  orb_slam3/src/KeyFrame.cc
  orb_slam3/src/Atlas.cc
  orb_slam3/src/Map.cc
  orb_slam3/src/MapDrawer.cc
  orb_slam3/src/Optimizer.cc
  orb_slam3/src/Frame.cc
  orb_slam3/src/KeyFrameDatabase.cc
  orb_slam3/src/Sim3Solver.cc
  orb_slam3/src/Viewer.cc
  orb_slam3/src/ImuTypes.cc
  orb_slam3/src/G2oTypes.cc
  orb_slam3/src/CameraModels/Pinhole.cpp
  orb_slam3/src/CameraModels/KannalaBrandt8.cpp
  orb_slam3/src/OptimizableTypes.cpp
  orb_slam3/src/MLPnPsolver.cpp
  orb_slam3/src/GeometricTools.cc
  orb_slam3/src/TwoViewReconstruction.cc
  orb_slam3/src/Config.cc
  orb_slam3/src/Settings.cc
  orb_slam3/include/System.h
  orb_slam3/include/Tracking.h
  orb_slam3/include/LocalMapping.h
  orb_slam3/include/LoopClosing.h
  orb_slam3/include/ORBextractor.h
  orb_slam3/include/ORBmatcher.h
  orb_slam3/include/FrameDrawer.h
  orb_slam3/include/Converter.h
  orb_slam3/include/MapPoint.h
  orb_slam3/include/KeyFrame.h
  orb_slam3/include/Atlas.h
  orb_slam3/include/Map.h
  orb_slam3/include/MapDrawer.h
  orb_slam3/include/Optimizer.h
  orb_slam3/include/Frame.h
  orb_slam3/include/KeyFrameDatabase.h
  orb_slam3/include/Sim3Solver.h
  orb_slam3/include/Viewer.h
  orb_slam3/include/ImuTypes.h
  orb_slam3/include/G2oTypes.h
  orb_slam3/include/CameraModels/GeometricCamera.h
  orb_slam3/include/CameraModels/Pinhole.h
  orb_slam3/include/CameraModels/KannalaBrandt8.h
  orb_slam3/include/OptimizableTypes.h
  orb_slam3/include/MLPnPsolver.h
  orb_slam3/include/GeometricTools.h
  orb_slam3/include/TwoViewReconstruction.h
  orb_slam3/include/SerializationUtils.h
  orb_slam3/include/Config.h
  orb_slam3/include/Settings.h
)

target_link_libraries(${PROJECT_NAME}_core
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${Pangolin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/DBoW2/lib/libDBoW2.so
  ${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/g2o/lib/libg2o.so
  -lboost_system
  -lboost_serialization
  -lcrypto
)

# Build ROS2 node for mono
add_executable(ros_mono
  src/ros_mono.cpp
  src/common.cpp
)

# Build ROS2 node for stereo
add_executable(ros_stereo 
  src/ros_stereo.cpp
  src/common.cpp
)

# Build ROS2 node for rgbd
add_executable(ros_rgbd 
  src/ros_rgbd.cpp
  src/common.cpp
)



# Generate service interfaces
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

# Link libraries
target_link_libraries(ros_mono
  ${PROJECT_NAME}_core
  "${cpp_typesupport_target}"
)

target_link_libraries(ros_stereo
  ${PROJECT_NAME}_core
  "${cpp_typesupport_target}"
)

target_link_libraries(ros_rgbd
  ${PROJECT_NAME}_core
  "${cpp_typesupport_target}"
)

# Add ROS2 dependencies to ros_mono
ament_target_dependencies(ros_mono
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  cv_bridge
  image_transport
  message_filters
  visualization_msgs
  rmw
  rmw_implementation
  rmw_fastrtps_cpp
)

# Add ROS2 dependencies to ros_stereo
ament_target_dependencies(ros_stereo
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  cv_bridge
  image_transport
  message_filters
  visualization_msgs
  rmw
  rmw_implementation
  rmw_fastrtps_cpp
)

# Add ROS2 dependencies to ros_rgbd
ament_target_dependencies(ros_rgbd
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  cv_bridge
  image_transport
  message_filters
  visualization_msgs
  rmw
  rmw_implementation
  rmw_fastrtps_cpp
)

# Install targets
install(TARGETS
  ${PROJECT_NAME}_core
  ros_mono
  ros_stereo
  ros_rgbd
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core)
ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  cv_bridge
  image_transport
  rmw
  rmw_implementation
  rmw_fastrtps_cpp
)

ament_package()