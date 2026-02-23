cmake_minimum_required(VERSION 3.16)
project(ros2_yolos_cpp)

# =============================================================================
# Build Settings
# =============================================================================
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# =============================================================================
# Dependencies
# =============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# ✅ REQUIRED FOR rosidl_generate_interfaces
find_package(rosidl_default_generators REQUIRED)

# =============================================================================
# FORCE SYSTEM ONNX RUNTIME (GPU BUILD)
# =============================================================================
set(ONNXRUNTIME_ROOT "/opt/onnxruntime-gpu")
set(ONNXRUNTIME_INCLUDE_DIR "${ONNXRUNTIME_ROOT}/include")
set(ONNXRUNTIME_LIB "${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so")

if(NOT EXISTS ${ONNXRUNTIME_LIB})
  message(FATAL_ERROR "GPU ONNX Runtime not found at ${ONNXRUNTIME_LIB}")
endif()

message(STATUS "Using GPU ONNX Runtime:")
message(STATUS "  Include: ${ONNXRUNTIME_INCLUDE_DIR}")
message(STATUS "  Library: ${ONNXRUNTIME_LIB}")

# =============================================================================
# YOLOs-CPP Download
# =============================================================================
set(YOLOS_CPP_VERSION "master")
set(YOLOS_CPP_DIR "${CMAKE_BINARY_DIR}/YOLOs-CPP")
set(YOLOS_CPP_URL "https://github.com/Geekgineer/YOLOs-CPP/archive/refs/heads/${YOLOS_CPP_VERSION}.tar.gz")

if(NOT EXISTS "${YOLOS_CPP_DIR}/include/yolos/yolos.hpp")
  message(STATUS "Downloading YOLOs-CPP...")

  file(DOWNLOAD
    "${YOLOS_CPP_URL}"
    "${CMAKE_BINARY_DIR}/yolos_cpp.tar.gz"
    SHOW_PROGRESS
  )

  execute_process(
    COMMAND ${CMAKE_COMMAND} -E tar xzf "${CMAKE_BINARY_DIR}/yolos_cpp.tar.gz"
    WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
  )

  file(GLOB EXTRACTED_DIRS "${CMAKE_BINARY_DIR}/YOLOs-CPP-*")
  list(GET EXTRACTED_DIRS 0 EXTRACTED_DIR)

  file(REMOVE_RECURSE "${YOLOS_CPP_DIR}")
  file(RENAME "${EXTRACTED_DIR}" "${YOLOS_CPP_DIR}")
endif()

message(STATUS "YOLOs-CPP Source: ${YOLOS_CPP_DIR}")

# =============================================================================
# Custom Messages
# =============================================================================
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/OrientedBoundingBox2D.msg"
  "msg/OBBDetection2D.msg"
  "msg/OBBDetection2DArray.msg"
  "msg/KeyPoint2D.msg"
  "msg/KeyPoint2DArray.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# =============================================================================
# Include Directories
# =============================================================================
include_directories(
  include
  ${YOLOS_CPP_DIR}/include
  ${ONNXRUNTIME_INCLUDE_DIR}
)

# =============================================================================
# Component Library
# =============================================================================
add_library(${PROJECT_NAME}_components SHARED
  src/adapters/detector_adapter.cpp
  src/adapters/segmentor_adapter.cpp
  src/adapters/pose_adapter.cpp
  src/adapters/obb_adapter.cpp
  src/adapters/classifier_adapter.cpp
  src/conversion/detection_converter.cpp
  src/conversion/segmentation_converter.cpp
  src/conversion/pose_converter.cpp
  src/conversion/obb_converter.cpp
  src/conversion/classification_converter.cpp
  src/nodes/detector_node.cpp
  src/nodes/segmentor_node.cpp
  src/nodes/pose_node.cpp
  src/nodes/obb_node.cpp
  src/nodes/classifier_node.cpp
)

target_link_libraries(${PROJECT_NAME}_components
  ${ONNXRUNTIME_LIB}
  ${OpenCV_LIBS}
)

ament_target_dependencies(${PROJECT_NAME}_components
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  image_transport
  cv_bridge
  sensor_msgs
  vision_msgs
  std_msgs
  geometry_msgs
  lifecycle_msgs
)

# Link generated message types
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}_components "${cpp_typesupport_target}")

# =============================================================================
# Register Components
# =============================================================================
rclcpp_components_register_node(${PROJECT_NAME}_components
  PLUGIN "ros2_yolos_cpp::YolosDetectorNode"
  EXECUTABLE yolos_detector_node
)

rclcpp_components_register_node(${PROJECT_NAME}_components
  PLUGIN "ros2_yolos_cpp::YolosSegmentorNode"
  EXECUTABLE yolos_segmentor_node
)

rclcpp_components_register_node(${PROJECT_NAME}_components
  PLUGIN "ros2_yolos_cpp::YolosPoseNode"
  EXECUTABLE yolos_pose_node
)

rclcpp_components_register_node(${PROJECT_NAME}_components
  PLUGIN "ros2_yolos_cpp::YolosOBBNode"
  EXECUTABLE yolos_obb_node
)

rclcpp_components_register_node(${PROJECT_NAME}_components
  PLUGIN "ros2_yolos_cpp::YolosClassifierNode"
  EXECUTABLE yolos_classifier_node
)

# =============================================================================
# RPATH for ONNX Runtime
# =============================================================================
get_filename_component(ONNXRUNTIME_LIB_DIR ${ONNXRUNTIME_LIB} DIRECTORY)
set_target_properties(${PROJECT_NAME}_components PROPERTIES
  BUILD_RPATH "${ONNXRUNTIME_LIB_DIR}"
  INSTALL_RPATH "${ONNXRUNTIME_LIB_DIR}"
)

# =============================================================================
# Install
# =============================================================================
install(TARGETS ${PROJECT_NAME}_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_components)
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  image_transport
  cv_bridge
  sensor_msgs
  vision_msgs
)

ament_package()

message(STATUS "")
message(STATUS "=== ros2_yolos_cpp GPU Configuration ===")
message(STATUS "  ONNX Runtime: ${ONNXRUNTIME_LIB}")
message(STATUS "  Build Type:   ${CMAKE_BUILD_TYPE}")
message(STATUS "")
