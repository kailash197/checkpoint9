cmake_minimum_required(VERSION 3.8)
project(my_components)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(composition REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(helper_package REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)
include_directories(${CMAKE_BINARY_DIR}/rosidl_generator_cpp)

# Generate ROSIDL interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GoToLoading.srv"
  DEPENDENCIES std_msgs
)

# Get ROSIDL typesupport target
rosidl_get_typesupport_target(attach_server_component_typesupport ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_get_typesupport_target(attach_client_component_typesupport ${PROJECT_NAME} "rosidl_typesupport_cpp")

### >> Add Component: preapproach
add_library(preapproach_component SHARED src/pre_approach.cpp)
target_compile_definitions(preapproach_component PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(preapproach_component
  "rclcpp"
  "rclcpp_components"
  "geometry_msgs"
  sensor_msgs
  nav_msgs
  tf2
  helper_package
  )
rclcpp_components_register_nodes(preapproach_component "my_components::PreApproach")
set(node_plugins "${node_plugins}my_components::PreApproach;$<TARGET_FILE:preapproach_component>\n")

### >> Add Component: attach_client
add_library(attach_client_component SHARED src/attach_client.cpp)
target_compile_definitions(attach_client_component PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(attach_client_component
  "rclcpp"
  "rclcpp_components"
  std_msgs
)
target_link_libraries(attach_client_component ${attach_client_component_typesupport})
rclcpp_components_register_nodes(attach_client_component "my_components::AttachClient")

### >> Add Component: attach_server
add_library(attach_server_component SHARED src/attach_server.cpp)
target_compile_definitions(attach_server_component PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(attach_server_component
  "rclcpp"
  "rclcpp_components"
  "geometry_msgs"
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  helper_package
)
target_link_libraries(attach_server_component ${attach_server_component_typesupport})
rclcpp_components_register_nodes(attach_server_component "my_components::AttachServer")

### >> Add Executable: manual_composition
add_executable(manual_composition src/manual_composition.cpp)
target_link_libraries(manual_composition attach_server_component)
ament_target_dependencies(manual_composition rclcpp geometry_msgs tf2_ros)

### >> Installations

install(TARGETS manual_composition
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  preapproach_component
  attach_server_component
  attach_client_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY rviz launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
