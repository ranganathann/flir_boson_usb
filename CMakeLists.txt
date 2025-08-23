cmake_minimum_required(VERSION 3.8)
project(flir_boson_usb)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Create BosonCamera component library
add_library(boson_camera SHARED
  src/nodelets/BosonCamera.cpp
)
target_compile_definitions(boson_camera
  PRIVATE "COMPOSITION_BUILDING_DLL"
)
ament_target_dependencies(boson_camera
  rclcpp
  rclcpp_components
  cv_bridge
  image_transport
  sensor_msgs
  camera_info_manager
)
target_link_libraries(boson_camera
  ${OpenCV_LIBRARIES}
)
rclcpp_components_register_nodes(boson_camera "flir_boson_usb::BosonCamera")

# Create standalone node
add_executable(flir_boson_usb_node
  src/flir_boson_usb_node.cpp
)
target_link_libraries(flir_boson_usb_node
  boson_camera
  ${OpenCV_LIBRARIES}
)
ament_target_dependencies(flir_boson_usb_node
  rclcpp
)

install(TARGETS
  boson_camera
  flir_boson_usb_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  example_calibrations
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()