cmake_minimum_required(VERSION 3.8)
project(nusim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nuturtlebot_msgs REQUIRED)
find_package(turtlelib REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}_srv
  "srv/Teleport.srv"
  LIBRARY_NAME ${PROJECT_NAME})

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME}_srv "rosidl_typesupport_cpp")

add_executable(nusim src/nusim.cpp)

ament_target_dependencies(nusim rclcpp)
ament_target_dependencies(nusim nav_msgs)
ament_target_dependencies(nusim std_msgs)
ament_target_dependencies(nusim sensor_msgs)
ament_target_dependencies(nusim std_srvs)
ament_target_dependencies(nusim geometry_msgs)
ament_target_dependencies(nusim tf2)
ament_target_dependencies(nusim tf2_ros)
ament_target_dependencies(nusim turtlelib)
ament_target_dependencies(nusim visualization_msgs)
ament_target_dependencies(nusim nuturtlebot_msgs)
ament_target_dependencies(nusim tf2_geometry_msgs)

target_link_libraries(nusim turtlelib::turtlelib "${cpp_typesupport_target}")

install(TARGETS
  nusim
  DESTINATION lib/nusim
)

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

# Install launch files.
install(DIRECTORY
  launch
  config
  DESTINATION share/nusim/
)

ament_package()
