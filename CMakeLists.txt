cmake_minimum_required(VERSION 3.5)

project(reference_system)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(reference_system_nodes SHARED
  src/arequipa.cpp
  src/barcelona.cpp
  src/geneva.cpp
  src/georgetown.cpp
  src/hamburg.cpp
  src/lyon.cpp
  src/mandalay.cpp
  src/monaco.cpp
  src/osaka.cpp
  src/ponce.cpp
  src/publisher_nodes.cpp
  src/rotterdam.cpp
  src/taipei.cpp
  src/tripoli.cpp
)

ament_target_dependencies(reference_system_nodes
  "rclcpp"
  "rclcpp_components"
  "geometry_msgs"
  "std_msgs"
  "sensor_msgs"
)

rclcpp_components_register_nodes(reference_system_nodes
  "reference_system::Arequipa"
  "reference_system::Barcelona"
  "reference_system::Cordoba"
  "reference_system::Delhi"
  "reference_system::Freeport"
  "reference_system::Geneva"
  "reference_system::Georgetown"
  "reference_system::Hamburg"
  "reference_system::Hebron"
  "reference_system::Kingston"
  "reference_system::Lyon"
  "reference_system::Osaka"
  "reference_system::Mandalay"
  "reference_system::Medellin"
  "reference_system::Monaco"
  "reference_system::Ponce"
  "reference_system::Portsmouth"
  "reference_system::Rotterdam"
  "reference_system::Taipei"
  "reference_system::Tripoli"
)

target_include_directories(reference_system_nodes PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

install(TARGETS
  reference_system_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
