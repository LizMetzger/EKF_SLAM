/// \file
/// \brief This node keeps track of the odometry of the robot and where it is
///
/// PARAMETERS:
///     \param odom_id the name of the odometry frame
///     \param body_id the name of the body frame
///     \param motor_cmd_max the maximum motor cmd
///     \param motor_cmd_per_rad_sec the conversion between motor cmd and rad/sec
///     \param encoder_ticks_per_rad the conversion between encoder tics and radians
///
/// PUBLISHES:
///     \param /wheel_cmd: publishes the position of the wheels
///     \param /joint_states: publishes joint states of the robot
///
/// SUBSCRIBES:
///     \param /cmd_vel: the velocity of the robot wheels
///     \param /sensor_data: the sensor data
///
/// SERVICES:
///     \param /initial_pose: reset the postion and oritentation of the robot to its starting one

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/exceptions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "turtlelib/diff_drive.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"


/// \brief a node for controling a turtlebot in simulation
class TurtleControl : public rclcpp::Node
{
public:
  TurtleControl()
  : Node("TurtleControl")
  {
    /// \brief get parameters from the yaml, throw an error if not defined
    declare_parameter("wheel_radius", -1.0);
    if (get_parameter("wheel_radius").get_parameter_value().get<double>() <= 0.0) {
      RCLCPP_ERROR_STREAM(get_logger(), "Please enter a valid wheel radius value");
      throw(std::logic_error("Please enter a valid wheel radius value"));
    } else {
      radius = get_parameter("wheel_radius").get_parameter_value().get<double>();
    }

    declare_parameter("track_width", -1.0);
    if (get_parameter("track_width").get_parameter_value().get<double>() <= 0.0) {
      RCLCPP_ERROR_STREAM(get_logger(), "Please enter a valid track width value");
      throw(std::logic_error("Please enter a valid track width value"));
    } else {
      track = get_parameter("track_width").get_parameter_value().get<double>();
    }

    declare_parameter("motor_cmd_max", -1.0);
    if (get_parameter("motor_cmd_max").get_parameter_value().get<double>() <= 0.0) {
      RCLCPP_ERROR_STREAM(get_logger(), "Please enter a valid motor cmd max value");
      throw(std::logic_error("Please enter a valid motor cmd max value"));
    } else {
      cmd_max = get_parameter("motor_cmd_max").get_parameter_value().get<double>();
    }

    declare_parameter("motor_cmd_per_rad_sec", -1.0);
    if (get_parameter("motor_cmd_per_rad_sec").get_parameter_value().get<double>() <= 0.0) {
      RCLCPP_ERROR_STREAM(get_logger(), "Please enter a valid motor cmd per rad sec value");
      throw(std::logic_error("Please enter a valid motor cmd per rad sec value"));
    } else {
      motor_cmd_per_rad_sec =
        get_parameter("motor_cmd_per_rad_sec").get_parameter_value().get<double>();
    }

    declare_parameter("encoder_ticks_per_rad", -1.0);
    if (get_parameter("encoder_ticks_per_rad").get_parameter_value().get<double>() <= 0.0) {
      RCLCPP_ERROR_STREAM(get_logger(), "Please enter a valid encoder ticks per rad value");
      throw(std::logic_error("Please enter a valid encoder ticks per rad value"));
    } else {
      encoder_ticks_per_rad =
        get_parameter("encoder_ticks_per_rad").get_parameter_value().get<double>();
    }

    /// \brief create a publisher for the wheel cmd
    /// topic: Nusim/wheel_cmd (nuturtlebot_msgs/msg/WheelCommands)
    wheel_cmd_publisher_ = create_publisher<nuturtlebot_msgs::msg::WheelCommands>(
      "/wheel_cmd", 10);

    /// \brief create a subscription to the twist
    /// topic: Nusim/cmd_vel (geometry_msgs/msg/Twist)
    cmd_vel_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&TurtleControl::cmd_vel_callback, this, std::placeholders::_1));

    /// \brief create a publisher for joint states
    /// topic: Nusim/wheel_cmd (nuturtlebot_msgs/msg/WheelCommands)
    joint_states_publisher_ = create_publisher<sensor_msgs::msg::JointState>(
      "/joint_states", 10);

    /// \brief create a subscription to sensor_data
    /// topic: Nusim/sensor_data (nuturtlebot_msgs::msg::SensorData)
    sensor_data_subscription_ = create_subscription<nuturtlebot_msgs::msg::SensorData>(
      "/sensor_data", 10,
      std::bind(&TurtleControl::sensor_data_callback, this, std::placeholders::_1));

    // initialize variables
    temp_rob = {track, radius};
  }

private:
  /// @brief callback that takes a twist and computes new wheel velocities
  /// @param msg - a twist
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // create a twist using the message
    turtlelib::Twist2D temp_twist = {msg->angular.z, {msg->linear.x, msg->linear.y}};

    // compute IK to get new wheel velocities
    turtlelib::Wheels new_wheel_velocity = temp_rob.calculate_IK(temp_twist);

    // get velocity in rad/s
    double temp_phi_l = new_wheel_velocity.phi_l / motor_cmd_per_rad_sec;
    double temp_phi_r = new_wheel_velocity.phi_r / motor_cmd_per_rad_sec;

    // create a wheel command message
    auto message = nuturtlebot_msgs::msg::WheelCommands();

    // constrain the wheel velocities
    if (temp_phi_l < -cmd_max) {
      temp_phi_l = -cmd_max;
    } else if (temp_phi_l > cmd_max) {
      temp_phi_l = cmd_max;
    }
    if (temp_phi_r < -cmd_max) {
      temp_phi_r = -cmd_max;
    } else if (temp_phi_r > cmd_max) {
      temp_phi_r = cmd_max;
    }

    // update the wheel velocites
    message.left_velocity = static_cast<int32_t>(temp_phi_l);
    message.right_velocity = static_cast<int32_t>(temp_phi_r);

    // publish the wheel commands
    wheel_cmd_publisher_->publish(message);
    return;
  }

  /// @brief callback that takes sensor data and computes joint states
  /// @param msg - sensor data
  void sensor_data_callback(const nuturtlebot_msgs::msg::SensorData::SharedPtr msg)
  {
    // get stamp from the sensor data
    joint_states.header.stamp = msg->stamp;
    joint_states.name = {"wheel_right_joint", "wheel_left_joint"};
    double curr_time = static_cast<double>(msg->stamp.sec + msg->stamp.nanosec * 1e-9);

    //the first time the callback runs
    if (first) {
      // set position and velocity to be zero
      joint_states.position = {0.0, 0.0};
      joint_states.velocity = {0.0, 0.0};
      first = false;

    }
    // every other time the callback runs
    else {
      // set the position to be what is read from sensor data in rads
      joint_states.position = {static_cast<double>(msg->right_encoder) / encoder_ticks_per_rad,
        static_cast<double>(msg->left_encoder) / encoder_ticks_per_rad};

      // find change in time
      double dt = curr_time - prev_time;

      // get velocity by dividing position by time
      // TODO: Store the last position so I can find change in position, then divide by dt
      joint_states.velocity = {joint_states.position.at(0) / dt,
        joint_states.position.at(1) / dt};

    }

    // save current time as previous time
    prev_time = curr_time;

    // publish the updates joint states
    joint_states_publisher_->publish(joint_states);
    return;
  }

  /// \brief initialize variables
  bool first = true;
  double radius;
  double track;
  double cmd_max;
  double motor_cmd_per_rad_sec;
  double encoder_ticks_per_rad;
  double prev_time = 0.0;
  sensor_msgs::msg::JointState prev_msg;
  sensor_msgs::msg::JointState joint_states;
  turtlelib::DiffDrive temp_rob = {track, radius};
  rclcpp::Publisher<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_publisher_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_subscription_;
};

/// \brief the main fucntion that spins the node
/// \param argc
/// \param argv
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<TurtleControl>());
  } catch (const std::exception &) {
    rclcpp::shutdown();
  }
  rclcpp::shutdown();
  return 0;
}
