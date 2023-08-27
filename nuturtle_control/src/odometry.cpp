/// \file
/// \brief This node keeps track of the odometry of the robot and where it is
///
/// PARAMETERS:
///     \param odom_id the name of the odometry frame
///     \param body_id the name of the body frame
///     \param wheel_left the left wheel of the robot
///     \param wheel_right the right wheel of the robot
///     \param track_width the width of the robot
///     \param wheel_radius the radius of the turtlebot wheels
///
/// PUBLISHES:
///     \param /odom: the odom frame of the blue robot
///     \param /path: publishes the path of the blue robot
///
/// SUBSCRIBES:
///     \param /joint_states: the joint staes of the robot
///
/// SERVICES:
///     \param /initial_pose: reset the postion and oritentation of the robot to its starting one


#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "turtlelib/diff_drive.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include "nuturtle_control/srv/initial_pose.hpp"

/// \brief a node for keeping track of the simulated position of the robot
class Odometry : public rclcpp::Node
{
public:
  Odometry()
  : Node("Odometry_node")
  {
    // get parameters from the launch and yaml file, exit if not defined
    declare_parameter("odom_id", "");
    if (get_parameter("odom_id").get_parameter_value().get<std::string>() == "") {
      throw(std::logic_error("Please enter a valid odom id value"));
    } else {
      odom_id = get_parameter("odom_id").get_parameter_value().get<std::string>();
    }

    declare_parameter("body_id", "");
    if (get_parameter("body_id").get_parameter_value().get<std::string>() == "") {
      throw(std::logic_error("Please enter a valid body_id value"));
    } else {
      body_id = get_parameter("body_id").get_parameter_value().get<std::string>();
    }

    declare_parameter("wheel_left", "");
    if (get_parameter("wheel_left").get_parameter_value().get<std::string>() == "") {
      throw(std::logic_error("Please enter a valid wheel_left value"));
    } else {
      wheel_left = get_parameter("wheel_left").get_parameter_value().get<std::string>();
    }

    declare_parameter("wheel_right", "");
    if (get_parameter("wheel_right").get_parameter_value().get<std::string>() == "") {
      throw(std::logic_error("Please enter a valid wheel_right value"));
    } else {
      wheel_right = get_parameter("wheel_right").get_parameter_value().get<std::string>();
    }

    declare_parameter("wheel_radius", -1.0);
    if (get_parameter("wheel_radius").get_parameter_value().get<double>() <= 0.0) {
      throw(std::logic_error("Please enter a valid wheel radius value"));
    } else {
      radius = get_parameter("wheel_radius").get_parameter_value().get<double>();
    }

    declare_parameter("track_width", -1.0);
    if (get_parameter("track_width").get_parameter_value().get<double>() <= 0.0) {
      throw(std::logic_error("Please enter a valid track width value"));
    } else {
      track = get_parameter("track_width").get_parameter_value().get<double>();
    }

    /// \brief create a subscription to joint_states
    /// topic: Odometry/joint_states (sensor_msgs::msg::JointState)
    joint_states_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&Odometry::joint_states_callback, this, std::placeholders::_1));

    /// \brief create a publisher for the odometry
    /// topic: /odom (nav_msgs::msg::Odometry)
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      "/odom", 10);

    /// \brief create a service to reset the robot to its initial pose
    /// service: /initial_pose (nuturtle_control::srv::InitialPos)
    initial_pose_service_ = create_service<nuturtle_control::srv::InitialPose>(
      "/initial_pose",
      std::bind(
        &Odometry::initial_pose, this,
        std::placeholders::_1, std::placeholders::_2));

    /// \brief create a publisher for the path
    /// topic: /red/path (nav_msgs/msg/Path)
    path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "/path", 10);

    // Initialize the transform broadcaster
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // initialize variables
    first = true;
    robot = turtlelib::DiffDrive{track, radius};
  }

private:
  /// @brief service call to reset the position of the robot to its initial pose
  /// @param request - a position and orientation
  /// @param  - empty
  void initial_pose(
    std::shared_ptr<nuturtle_control::srv::InitialPose::Request> request,
    std::shared_ptr<nuturtle_control::srv::InitialPose::Response>)
  {
    double temp_phi_l = robot.get_wheels().phi_l;
    double temp_phi_r = robot.get_wheels().phi_r;
    robot =
      turtlelib::DiffDrive{track, radius, {temp_phi_r, temp_phi_l},
      {request->theta, request->x, request->y}};
    first = false;
    return;
  }

  /// @brief call back that takes in joint states to publish a transform of the
  /// new position of the robot
  /// @param msg - joint states message
  void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // get right and left wheel positions from joint state publisher
    double pos_right = msg->position[0];
    double pos_left = msg->position[1];

    // set the new position of the wheels
    turtlelib::Wheels wheel_delta = {pos_right - robot.get_wheels().phi_r,
      pos_left - robot.get_wheels().phi_l};

    // compute a new body twist to publish
    twist = robot.calc_bod_twist(wheel_delta);

    // calculate FK with the new wheel positions
    robot.calculate_FK(wheel_delta);
    tf2_quat.setRPY(0.0, 0.0, robot.get_config().w);

    // use the calculated values to publish an updated odometry message
    auto message = nav_msgs::msg::Odometry();
    message.header.stamp = msg->header.stamp;
    message.header.frame_id = odom_id;
    message.child_frame_id = body_id;
    message.pose.pose.position.x = robot.get_config().x;
    message.pose.pose.position.y = robot.get_config().y;
    message.pose.pose.position.z = 0.0;
    message.pose.pose.orientation.x = tf2_quat.x();
    message.pose.pose.orientation.y = tf2_quat.y();
    message.pose.pose.orientation.z = tf2_quat.z();
    message.pose.pose.orientation.w = tf2_quat.w();
    message.twist.twist.angular.z = twist.getAng();
    message.twist.twist.linear.x = twist.getTran().x;
    message.twist.twist.linear.y = twist.getTran().y;
    // publish the message
    odom_publisher_->publish(message);

    // publish the transform between the body of the robot and the odom
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = msg->header.stamp;
    t.header.frame_id = odom_id;
    t.child_frame_id = body_id;
    t.transform.translation.x = robot.get_config().x;
    t.transform.translation.y = robot.get_config().y;
    t.transform.translation.z = 0.0;
    t.transform.rotation.x = tf2_quat.x();
    t.transform.rotation.y = tf2_quat.y();
    t.transform.rotation.z = tf2_quat.z();
    t.transform.rotation.w = tf2_quat.w();
    // Send the transformation
    tf_broadcaster_->sendTransform(t);

    // create a new pose to add to the path
    geometry_msgs::msg::PoseStamped new_pose;
    new_pose.header = t.header;
    new_pose.pose.position.x = t.transform.translation.x;
    new_pose.pose.position.y = t.transform.translation.y;
    new_pose.pose.orientation.x = t.transform.rotation.x;
    new_pose.pose.orientation.y = t.transform.rotation.y;
    new_pose.pose.orientation.z = t.transform.rotation.z;
    new_pose.pose.orientation.w = t.transform.rotation.w;

    // create a path to follow
    path.header = t.header;
    path.poses.push_back(new_pose);

    // publish the path
    path_publisher_->publish(path);
    return;
  }

  /// \brief initialize variables
  double track, radius;
  bool first;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Service<nuturtle_control::srv::InitialPose>::SharedPtr initial_pose_service_;
  geometry_msgs::msg::Quaternion msg_quat;
  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::Path path;
  std::string odom_id;
  std::string body_id;
  std::string wheel_left;
  std::string wheel_right;
  turtlelib::Twist2D twist;
  turtlelib::DiffDrive robot = {track, radius};
  tf2::Quaternion tf2_quat;

};

/// \brief the main fucntion that spins the node
/// \param argc
/// \param argv
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<Odometry>());
  } catch (const std::exception &) {
    rclcpp::shutdown();
  }
  rclcpp::shutdown();
  return 0;
}
