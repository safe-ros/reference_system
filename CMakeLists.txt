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
  src/cordoba.cpp
  src/delhi.cpp
  src/freeport.cpp
  src/geneva.cpp
  src/georgetown.cpp
  src/hamburg.cpp
  src/hebron.cpp
  src/kingston.cpp
  src/lyon.cpp
  src/mandalay.cpp
  src/medellin.cpp
  src/monaco.cpp
  src/osaka.cpp
  src/ponce.cpp
  src/portsmouth.cpp
  src/rotterdam.cpp
  src/taipei.cpp
  src/tripoli.cpp
  src/utils.cpp
)

ament_target_dependencies(reference_system_nodes
  "rclcpp"
  "rclcpp_components"
  "geometry_msgs"
  "std_msgs"
  "sensor_msgs"
)

set(NODES
  "Arequipa"
  "Barcelona"
  "Cordoba"
  "Delhi"
  "Freeport"
  "Geneva"
  "Georgetown"
  "Hamburg"
  "Hebron"
  "Kingston"
  "Lyon"
  "Osaka"
  "Mandalay"
  "Medellin"
  "Monaco"
  "Ponce"
  "Portsmouth"
  "Rotterdam"
  "Taipei"
  "Tripoli"
)

foreach(NODE ${NODES})
  string(TOLOWER ${NODE} NODE_LOWER)

  rclcpp_components_register_node(reference_system_nodes
    PLUGIN "reference_system::${NODE}"
    EXECUTABLE ${NODE_LOWER}
  )
endforeach()

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
