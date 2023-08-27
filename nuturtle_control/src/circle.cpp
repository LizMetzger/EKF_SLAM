/// \file
/// \brief This node moved the robot in a circle in rviz
///
/// PARAMETERS:
///     \param freq the rate in Hertz of the timercallback
///
/// PUBLISHES:
///     \param /cmd_vel: publishes the cmd vel of the robot
///
/// SERVICES:
///     \param ~/reset: reset the postion and oritentation of the robot to its starting one
///     \param ~/teleport: teleport the robot to a designated location
///     \param ~/teleport: teleport the robot to a designated location

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlelib/diff_drive.hpp"
#include "std_srvs/srv/empty.hpp"
#include "nuturtle_control/srv/control.hpp"

/// \brief a node for controling a turtlebot in simulation
class Circle : public rclcpp::Node
{
public:
  Circle()
  : Node("Circle_node")
  {
    // get the timer frequency from the yaml
    declare_parameter("freq", 100.0);
    double rate = get_parameter("freq").get_parameter_value().get<double>(); //! const auto
    rate = (1.0 / rate) * 1000; //! 1.0/rate,

    /// \brief create a timer to run the timercallback
    /// \param rate - the rate in ms
    timer_ = create_wall_timer(
      std::chrono::milliseconds(long(rate)),
      std::bind(&Circle::timerCallback, this));

    /// \brief create a publisher for the wheel cmd
    /// topic: Nusim/wheel_cmd (nuturtlebot_msgs/msg/WheelCommands)
    cmd_vel_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10);

    /// \brief create a service for teleporting the robot
    /// service: Odometry/initial_pose (nusim::srv::Teleport)
    control_service_ = create_service<nuturtle_control::srv::Control>(
      "/control",
      std::bind(&Circle::control, this, std::placeholders::_1, std::placeholders::_2));

    /// \brief create a service for reverse the direction of the circle
    /// service: Nusim/reverse (std_srvs::srv::Empty)
    reverse_service_ = create_service<std_srvs::srv::Empty>(
      "/reverse",
      std::bind(&Circle::reverse, this, std::placeholders::_1, std::placeholders::_2));

    /// \brief create a service for resetting the robot
    /// service: Nusim/reset (std_srvs::srv::Empty)
    stop_service_ = create_service<std_srvs::srv::Empty>(
      "/stop",
      std::bind(&Circle::stop, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  /// @brief service call to ser a radius and velocity to follow
  /// @param request - a velocity and a radius
  /// @param response - empty
  void control(
    std::shared_ptr<nuturtle_control::srv::Control::Request> request,
    std::shared_ptr<nuturtle_control::srv::Control::Response>)
  {
    w = request->w;
    radius = request->r;
    pub = true;
    return;
  }

  /// @brief service to reverse the direction of the robot's circle
  /// @param  - empty
  /// @param  - empty
  void reverse(
    std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    w = -w;
    pub = true;
    return;
  }

  /// @brief service call to stop the robot
  /// @param  - empty
  /// @param  - empty
  void stop(
    std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    w = 0.0;
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 0.0;
    message.linear.y = 0.0;
    message.angular.z = 0.0;
    cmd_vel_publisher_->publish(message);
    pub = false;
    return;
  }

  /// \brief main timercallback that publishes and broadcasts
  void timerCallback()
  {
    if (pub) {
      auto message = geometry_msgs::msg::Twist();
      message.linear.x = w * radius;
      message.linear.y = 0.0;
      message.angular.z = w;
      cmd_vel_publisher_->publish(message);
    }
  }

  /// \brief initialize variables
  bool pub = false;
  rclcpp::TimerBase::SharedPtr timer_;
  double w = 0.0;
  double radius = 0.0;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::Service<nuturtle_control::srv::Control>::SharedPtr control_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reverse_service_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_service_;

};

/// \brief the main fucntion that spins the node
/// \param argc
/// \param argv
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Circle>());
  rclcpp::shutdown();
  return 0;
}
