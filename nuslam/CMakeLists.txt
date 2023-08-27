cmake_minimum_required(VERSION 3.8)
project(nuslam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(Armadillo REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nuturtlebot_msgs REQUIRED)
find_package(turtlelib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)


add_executable(nuslam src/nuslam.cpp)
add_executable(landmarks src/landmarks.cpp)
ament_target_dependencies(nuslam 
                          rclcpp
                          nav_msgs
                          std_msgs
                          sensor_msgs
                          geometry_msgs
                          tf2
                          tf2_ros
                          turtlelib
                          nuturtlebot_msgs
                          tf2_geometry_msgs
                          visualization_msgs)

ament_target_dependencies(landmarks
                          rclcpp
                          nav_msgs
                          std_msgs
                          sensor_msgs
                          geometry_msgs
                          tf2
                          tf2_ros
                          turtlelib
                          nuturtlebot_msgs
                          tf2_geometry_msgs
                          visualization_msgs)


target_link_libraries(nuslam turtlelib::turtlelib ${ARMADILLO_LIBRARIES} "${cpp_typesupport_target}")
target_link_libraries(landmarks turtlelib::turtlelib "${cpp_typesupport_target}")

target_include_directories(nuslam PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(nuslam PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS nuslam
                landmarks
  DESTINATION lib/${PROJECT_NAME})

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
  DESTINATION share/nuslam/
)

ament_package()
