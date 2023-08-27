/// \file
/// \brief This node simulates a turtlebot navigating in an arena with obstacles in rviz
///
/// PARAMETERS:
///     \param rate the rate in Hertz of the timercallback
///     \param x the x coordinate of the turtlebot
///     \param y the y coordinate of the turtlebot
///     \param theta the theta value of the turtlebot
///     \param collision_radius the collision radius of the turtlebot
///     \param obs_x a vector of obstacle x coordinates
///     \param obs_y a vector of obstacle y coordinates
///     \param radius the radius of the obstacles
///     \param x_length the number of x coordinates given in obs_x
///     \param y_length the number of y coordinates given in obs_y
///     \param input_noise the noise of the simulated robot
///     \param slip_fraction the amount that the wheels of the real robot slip
///     \param basic_sensor_variance the variancce of the sensor data
///     \param max_range the max range of the fake sensor
///     \param laser_noise the noise in the laser scan
///     \param min_laser_range the minimum range of the laser scan
///     \param max_laser_range the maximum range of the laser scan
///     \param angle_increment the amount that the laser scan increments by
///     \param draw true or false for wether or not to draw or simulate the obstacles
///     \param track_width the width of the robot
///     \param wheel_radius the radius of the turtlebot wheels
///     \param encoder_tics_per_rad the conversion between encoder tics and radians
///     \param motor_cmd_per_rad_sec the conversion between cmd and rad per sec
///
/// PUBLISHES:
///     \param ~/time_step: publishes the current timestep
///     \param ~/obstacles: publishes the obstacles to rviz
///     \param ~/walls: publishes the walls of the arena in rviz
///     \param /red/sensor_data: publishes sensor data from the red robot
///     \param /red/path: publishes the path that the red robot has taken to rviz
///     \param /red/laser_scan: publishes the simulated laser scan data
///     \param ~/fake_sensor: published the fake scan data of markers
///
/// SUBSCRIBES:
///     \param /red/wheel_cmd: subscribes to the red robot's cmd vel
///
/// SERVICES:
///     \param ~/reset: reset the postion and oritentation of the robot to its starting one
///     \param ~/teleport: teleport the robot to a designated location


#include <cmath>
#include <chrono>
#include <memory>
#include <string>
#include <random>
#include <functional>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


#include "rclcpp/rclcpp.hpp"
#include "rclcpp/exceptions.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "nusim/srv/teleport.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/make_circles.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

/// \brief a node for controling a turtlebot in simulation
class Nusim : public rclcpp::Node
{
public:
  Nusim()
  : Node("Nusim_node"), count_(0)
  {
    // get parameters from the yaml
    declare_parameter("rate", 200.0);
    double rate = get_parameter("rate").get_parameter_value().get<double>();
    dt = (1.0 / rate);
    declare_parameter("x", 0.0);
    declare_parameter("y", 0.0);
    declare_parameter("theta", 0.0);
    declare_parameter("collision_radius", 0.0);
    x_0 = get_parameter("x").get_parameter_value().get<double>();
    y_0 = get_parameter("y").get_parameter_value().get<double>();
    theta_0 = get_parameter("theta").get_parameter_value().get<double>();
    collision_radius = get_parameter("collision_radius").get_parameter_value().get<double>();
    declare_parameter("obstacles.x", std::vector<double>{});
    declare_parameter("obstacles.y", std::vector<double>{});
    declare_parameter("obstacles.r", 0.01);
    obs_x = get_parameter("obstacles.x").get_parameter_value().get<std::vector<double>>();
    obs_y = get_parameter("obstacles.y").get_parameter_value().get<std::vector<double>>();
    radius = get_parameter("obstacles.r").get_parameter_value().get<double>();
    declare_parameter("x_length", 0.0);
    declare_parameter("y_length", 0.0);
    x_length = get_parameter("x_length").get_parameter_value().get<double>();
    y_length = get_parameter("y_length").get_parameter_value().get<double>();
    declare_parameter("input_noise", 0.0);
    declare_parameter("slip_fraction", 0.0);
    declare_parameter("basic_sensor_variance", 0.0);
    declare_parameter("max_range", 0.0);
    input_noise = get_parameter("input_noise").get_parameter_value().get<double>();
    slip_fraction = get_parameter("slip_fraction").get_parameter_value().get<double>();
    basic_sensor_variance =
      get_parameter("basic_sensor_variance").get_parameter_value().get<double>();
    max_range = get_parameter("max_range").get_parameter_value().get<double>();

    // laser parameters
    declare_parameter("num_samples", 0);
    declare_parameter("laser_noise", 0.0);
    declare_parameter("min_laser_range", 0.0);
    declare_parameter("max_laser_range", 0.0);
    declare_parameter("angle_increment", 0.0);
    num_samples = get_parameter("num_samples").get_parameter_value().get<int>();
    laser_noise = get_parameter("laser_noise").get_parameter_value().get<double>();
    min_laser_range = get_parameter("min_laser_range").get_parameter_value().get<double>();
    max_laser_range = get_parameter("max_laser_range").get_parameter_value().get<double>();
    angle_increment = get_parameter("angle_increment").get_parameter_value().get<double>();

    // draw_only parameter
    declare_parameter("draw", false);
    draw_only = get_parameter("draw").get_parameter_value().get<bool>();

    // load parameters from the launchfile or command line, close if not given
    declare_parameter("wheel_radius", -1.0);
    if (get_parameter("wheel_radius").get_parameter_value().get<double>() <= 0.0) {
      RCLCPP_ERROR_STREAM(get_logger(), "Please enter a valid wheel radius value");
      throw(std::logic_error("Please enter a valid wheel radius value"));
    } else {
      wheel_radius = get_parameter("wheel_radius").get_parameter_value().get<double>();
    }

    declare_parameter("track_width", -1.0);
    if (get_parameter("track_width").get_parameter_value().get<double>() <= 0.0) {
      RCLCPP_ERROR_STREAM(get_logger(), "Please enter a valid track width value");
      throw(std::logic_error("Please enter a valid track width value"));
    } else {
      track = get_parameter("track_width").get_parameter_value().get<double>();
    }

    declare_parameter("encoder_ticks_per_rad", -1.0);
    if (get_parameter("encoder_ticks_per_rad").get_parameter_value().get<double>() <= 0.0) {
      RCLCPP_ERROR_STREAM(get_logger(), "Please enter a valid encoder ticks per rad value");
      throw(std::logic_error("Please enter a valid encoder ticks per rad value"));
    } else {
      encoder_ticks_per_rad =
        get_parameter("encoder_ticks_per_rad").get_parameter_value().get<double>();
    }

    declare_parameter("motor_cmd_per_rad_sec", -1.0);
    if (get_parameter("motor_cmd_per_rad_sec").get_parameter_value().get<double>() <= 0.0) {
      RCLCPP_ERROR_STREAM(get_logger(), "Please enter a valid motor cmd per rad sec value");
      throw(std::logic_error("Please enter a valid motor cmd per rad sec value"));
    } else {
      motor_cmd_per_rad_sec =
        get_parameter("motor_cmd_per_rad_sec").get_parameter_value().get<double>();
    }

    /// \brief create a timer to run the timercallback
    /// \param dt - the rate in ms
    timer_ = create_wall_timer(
      std::chrono::milliseconds(long(dt * 1000)),
      std::bind(&Nusim::timerCallback, this));

    /// \brief create a timer to run the timercallback2
    /// \param rate - the rate in ms
    timer2_ = create_wall_timer(
      std::chrono::milliseconds(long(1000 / 5)),
      std::bind(&Nusim::fakeSensorCallback, this));

    /// \brief create a publisher for the timestep
    /// topic: Nusim/time_step (std_msgs/msg/UInt64)
    publisher_ = create_publisher<std_msgs::msg::UInt64>(
      "~/time_step", 10);

    /// \brief create a publisher for the obstacle markers
    /// topic: Nusim/obstacles (visualization_msgs/msg/MarkerArray)
    obstacle_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/obstacles", 10);

    /// \brief create a publisher for the owall markers
    /// topic: Nusim/walls (visualization_msgs/msg/MarkerArray)
    wall_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/walls", 10);

    /// \brief create a publisher for the wheel cmd
    /// topic: /red/wheel_cmd (nuturtlebot_msgs/msg/WheelCommands)
    sensor_data_publisher_ = create_publisher<nuturtlebot_msgs::msg::SensorData>(
      "/red/sensor_data", 10);

    /// \brief create a publisher for the path
    /// topic: /red/path (nav_msgs/msg/Path)
    path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "/red/path", 10);

    /// \brief create a publisher for the laser data
    /// topic: /red/path (sensor_msgs/msg/LaserScan)
    laser_scan_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(
      "/red/laser_scan", 10);

    /// \brief create a service for resetting the robot
    /// service: Nusim/reset (std_srvs::srv::Empty)
    service_ = create_service<std_srvs::srv::Empty>(
      "~/reset",
      std::bind(&Nusim::reset, this, std::placeholders::_1, std::placeholders::_2));

    /// \brief create a service for teleporting the robot
    /// service: Nusim/teleport (nusim::srv::Teleport)
    teleport_service_ = create_service<nusim::srv::Teleport>(
      "~/teleport",
      std::bind(&Nusim::teleport, this, std::placeholders::_1, std::placeholders::_2));

    /// \brief create a broadcaster for "nusim/world" to "red/base_footprint"
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    /// \brief create a subscription to red/wheel_cmd
    /// topic: /red/cmd_vel (geometry_msgs/msg/Twist)
    wheel_cmd_subscription_ = create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
      "/red/wheel_cmd", 10, std::bind(&Nusim::wheel_cmd_callback, this, std::placeholders::_1));

    /// \brief create a publisher for the obstacle markers
    /// topic: Nusim/obstacles (visualization_msgs/msg/MarkerArray)
    fake_sensor_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/fake_sensor", 10);

    red_robot = turtlelib::DiffDrive{track, wheel_radius};

    /// \brief temp variables for reset service
    x_init = x_0;
    y_init = y_0;
    theta_init = theta_0;

    // obstacle_timer = get_clock()->now();
    /// \brief loop to fill the marker array with any defined obstacles
    if (std::size(obs_x) == std::size(obs_y)) {
      for (int i = 0; i < (int)std::size(obs_x); i++) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "nusim/world";
        marker.header.stamp = get_clock()->now();
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.pose.position.x = obs_x[i];
        marker.pose.position.y = obs_y[i];
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.id = i;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.scale.x = radius;
        marker.scale.y = radius;
        marker.scale.z = 0.25;
        obs.markers.push_back(marker);
      }
    } else {
      RCLCPP_ERROR(get_logger(), "NODE SHUTTING DOWN: INVALID ARRAY ENTRIES!!!");
      throw(std::logic_error("NODE SHUTTING DOWN: INVALID ARRAY ENTRIES!!!"));
    }

    // variables to generate walls
    wall_thickness = .1;
    std::vector<std::vector<double>> wall_pos = {{(x_length / 2) + (wall_thickness / 2), 0.0},
      {0.0, (y_length / 2) + (wall_thickness / 2)},
      {-((x_length / 2) + (wall_thickness / 2)), 0.0},
      {0.0, -((y_length / 2) + (wall_thickness / 2))}};
    std::vector<std::vector<double>> wall_dim = {{wall_thickness, y_length},
      {x_length, wall_thickness},
      {wall_thickness, y_length},
      {x_length, wall_thickness}};

    /// \brief loop to fill the marker array with all walls
    for (int i = 0; i < (int)std::size(wall_pos); i++) {
      visualization_msgs::msg::Marker wall;
      wall.header.frame_id = "nusim/world";
      wall.header.stamp = get_clock()->now();
      wall.action = visualization_msgs::msg::Marker::ADD;
      wall.type = visualization_msgs::msg::Marker::CUBE;
      wall.pose.position.x = wall_pos[i][0];
      wall.pose.position.y = wall_pos[i][1];
      wall.pose.position.z = 0.0;
      wall.pose.orientation.x = 0.0;
      wall.pose.orientation.y = 0.0;
      wall.pose.orientation.z = 0.0;
      wall.pose.orientation.w = 1.0;
      wall.id = i;
      wall.color.a = 1.0;
      wall.color.r = 0.75;
      wall.color.g = 0.0;
      wall.color.b = 1.0;
      wall.scale.x = wall_dim[i][0];
      wall.scale.y = wall_dim[i][1];
      wall.scale.z = 0.25;
      walls.markers.push_back(wall);
    }

    /// \brief create a subscription to red/wheel_cmd
    /// topic: Nusim/cmd_vel (geometry_msgs/msg/Twist)
    // wheel_cmd_subscription_ = create_subscription<nuturtlebot_msgs::msg::WheelCommands>(
    //   "/red/wheel_cmd", 10, std::bind(&Nusim::wheel_cmd_callback, this, std::placeholders::_1));

    /// \brief create a publisher for the wheel cmd
    /// topic: Nusim/wheel_cmd (nuturtlebot_msgs/msg/WheelCommands)
    // sensor_data_publisher_ = create_publisher<nuturtlebot_msgs::msg::SensorData>(
    //   "/red/sensor_data", 10);

    red_robot = turtlelib::DiffDrive{track, wheel_radius};

    // create a random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
  }

private:
  /// \brief main timercallback that publishes and broadcasts
  void timerCallback()
  {
    timestamp = get_clock()->now();
    /// publish and increnet the timestep counter
    auto message = std_msgs::msg::UInt64();
    message.data = count_;
    publisher_->publish(message);
    count_++;

    // ONLY PASSING THE CHANGE IN MY POSITION TO IK NOT MY WHOLE POSITION
    delta_wheels = turtlelib::Wheels{dt * phi_r_rad_s, dt * phi_l_rad_s};

    // do forward kinematics to get a twist to get to the new wheel positions
    red_robot.calculate_FK(delta_wheels);

    // after publishing new wheel positions, but before braodcasting the new x/y position
    // check to see if the robot has collided with an obstacle
    if (!draw_only) {
      for (int j = 0; j < (int)std::size(obs_x); j++) {
        double x_diff = red_robot.get_config().x - obs.markers.at(j).pose.position.x;
        double y_diff = red_robot.get_config().y - obs.markers.at(j).pose.position.y;
        double dist = sqrt((x_diff * x_diff) + (y_diff * y_diff));
        //if it has, move its x, y position to be outside the obstacle to its tangent
        double total_rad = radius + collision_radius;
        if (dist <= total_rad) {
          red_robot.set_x(red_robot.get_config().x + (((total_rad) - dist) / (total_rad)) * x_diff);
          red_robot.set_y(red_robot.get_config().y + (((total_rad) - dist) / (total_rad)) * y_diff);
        }
      }
    }

    // get clock stamp for sensor data
    sensor_readings.stamp = timestamp;

    // publish the new wheel positions as sensor readings
    sensor_readings.left_encoder = (red_robot.get_wheels().phi_l - dt * phi_l_rad_s) * encoder_ticks_per_rad;
    sensor_readings.right_encoder = (red_robot.get_wheels().phi_r - dt * phi_r_rad_s) * encoder_ticks_per_rad;

    // create transform to be braodcasted
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = timestamp;
    t.header.frame_id = "nusim/world";
    t.child_frame_id = "red/base_footprint";
    t.transform.translation.x = red_robot.get_config().x;
    t.transform.translation.y = red_robot.get_config().y;
    t.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, red_robot.get_config().w);
    geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(q);
    t.transform.rotation.x = msg_quat.x;
    t.transform.rotation.y = msg_quat.y;
    t.transform.rotation.z = msg_quat.z;
    t.transform.rotation.w = msg_quat.w;

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
    if (count_ == 10) {
      path.poses.push_back(new_pose);
      // publish the path
      path_publisher_->publish(path);
      count_ = 0;
    }

    // publish sensor readings
    sensor_data_publisher_->publish(sensor_readings);
  }

  /// @brief callback that takes in wheel commands and uses them to calculate the new postion of the robot
  /// @param msg
  void wheel_cmd_callback(const nuturtlebot_msgs::msg::WheelCommands::SharedPtr msg)
  {
    // generate a random number from the normal distribution
    std::normal_distribution<double> dist(0.0, input_noise);

    // generate a random number from a uniform random sample
    std::uniform_real_distribution<double> uniform_dist(-slip_fraction, slip_fraction);

    // get the left and right wheel in rad/s (go from ticks to rads)
    no_noise_phi_l_rad_s = msg->left_velocity * motor_cmd_per_rad_sec;
    no_noise_phi_r_rad_s = msg->right_velocity * motor_cmd_per_rad_sec;

    // add random noise to each of the wheels as long as they are moving
    if (no_noise_phi_l_rad_s != 0.0) {
      phi_l_rad_s = no_noise_phi_l_rad_s + dist(gen);
    } else {
      phi_l_rad_s = no_noise_phi_l_rad_s;
    }
    if (no_noise_phi_r_rad_s != 0.0) {
      phi_r_rad_s = no_noise_phi_r_rad_s + dist(gen);
    } else {
      phi_r_rad_s = no_noise_phi_r_rad_s;
    }

    // // add slippage to the wheels
    red_robot.set_l_wheel(red_robot.get_wheels().phi_l * (1 + uniform_dist(gen)));
    red_robot.set_r_wheel(red_robot.get_wheels().phi_r * (1 + uniform_dist(gen)));
    return;
  }

  /// @brief
  void fakeSensorCallback()
  {
    // generate a random number from the normal distribution
    std::normal_distribution<double> marker_dist(0.0, basic_sensor_variance);
    visualization_msgs::msg::MarkerArray fake_obs;
    // generate fake markers to represnet the sensor data
    for (int i = 0; i < (int)std::size(obs_x); i++) {
      turtlelib::Transform2D T_wobs{{obs_x[i], obs_y[i]}, 0.0};
      turtlelib::Transform2D T_wr{{red_robot.get_config().x, red_robot.get_config().y},
        red_robot.get_config().w};
      turtlelib::Transform2D T_robs = T_wr.inv() * T_wobs;

      visualization_msgs::msg::Marker fake_marker;
      fake_marker.header.frame_id = "red/base_footprint";
      fake_marker.header.stamp = get_clock()->now();
      fake_marker.action = visualization_msgs::msg::Marker::ADD;
      fake_marker.type = visualization_msgs::msg::Marker::CYLINDER;
      fake_marker.pose.position.x = T_robs.translation().x + marker_dist(gen);
      fake_marker.pose.position.y = T_robs.translation().y + marker_dist(gen);
      fake_marker.pose.position.z = 0.0;
      fake_marker.pose.orientation.x = 0.0;
      fake_marker.pose.orientation.y = 0.0;
      fake_marker.pose.orientation.z = 0.0;
      fake_marker.pose.orientation.w = 1.0;
      fake_marker.id = i;
      fake_marker.color.a = 1.0;
      fake_marker.color.r = 1.0;
      fake_marker.color.g = 1.0;
      fake_marker.color.b = 0.3;
      fake_marker.scale.x = radius;
      fake_marker.scale.y = radius;
      fake_marker.scale.z = 0.25;


      double d =
        (fake_marker.pose.position.x * fake_marker.pose.position.x + fake_marker.pose.position.y *
        fake_marker.pose.position.y) - radius;
      // if its not within range set its action to delete
      if (d > max_range * max_range) {
        fake_marker.action = visualization_msgs::msg::Marker::DELETE;
      }
      // if it is then set it to add
      else {
        fake_marker.action = visualization_msgs::msg::Marker::ADD;
      }
      fake_obs.markers.push_back(fake_marker);
    }

    // publish all of the fake obstacles
    fake_sensor_publisher_->publish(fake_obs);

    // create gaussian noise for the laser
    std::normal_distribution<double> laser_var(0.0, laser_noise);

    // fill the laser scan message
    sensor_msgs::msg::LaserScan laser;
    laser.header.stamp = timestamp;
    // laser.header.stamp.sec -= 0.2;
    laser.header.frame_id = "red/base_scan";
    laser.angle_min = 0.0;
    laser.angle_max = 6.2657318;
    laser.angle_increment = angle_increment;
    laser.time_increment = 0.0;
    laser.scan_time = 0.2006689;
    laser.range_min = min_laser_range;
    laser.range_max = max_laser_range;

    // for each increment see if it hits a marker
    for (int k = 0; k < num_samples; k++) {
      double min_dist = laser.range_max + 1.0;
      double theta_laser = laser.angle_increment * k;
      // find the cooridnates of the max point
      double max_x = red_robot.get_config().x + std::cos(theta_laser + red_robot.get_config().w) *
        laser.range_max;
      double max_y = red_robot.get_config().y + std::sin(theta_laser + red_robot.get_config().w) *
        laser.range_max;
      // get the slope of the line between the max point and the robot
      double m = (max_y - red_robot.get_config().y) / (max_x - red_robot.get_config().x);

      // check each marker for a collision
      for (int i = 0; i < (int)std::size(obs_x); i++) {
        double alpha = red_robot.get_config().y - m * red_robot.get_config().x -
          obs.markers.at(i).pose.position.y;
        double A = (1 + m * m);
        double B = 2 * (-obs.markers.at(i).pose.position.x + alpha * m);
        double C = (obs.markers.at(i).pose.position.x * obs.markers.at(i).pose.position.x) +
          (alpha * alpha) - (radius * radius);
        double determinate = B * B - 4 * A * C;
        // if the discriminate is greater than 0, there are two intersections, pick the one closest to the robot
        if (determinate > 0.0) {
          double x1 = (-B + sqrt(determinate)) / (2 * A);
          double y1 = m * (x1 - red_robot.get_config().x) + red_robot.get_config().y;
          double dist1 = turtlelib::get_dist(red_robot.get_config().x, red_robot.get_config().y, x1, y1);
          double x2 = (-B - sqrt(determinate)) / (2 * A);
          double y2 = m * (x2 - red_robot.get_config().x) + red_robot.get_config().y;
          double dist2 = turtlelib::get_dist(red_robot.get_config().x, red_robot.get_config().y, x2, y2);
          // if point one is closer and valid then add it
          if (dist1 < dist2 && dist1 < min_dist &&
            turtlelib::check_line(
              x1, red_robot.get_config().x,
              max_x) > 0.0 && turtlelib::check_line(y1, red_robot.get_config().y, max_y) > 0.0)
          {
            min_dist = dist1;
          }
          // otherwise if point two is closer and valid add it
          else if (dist2 < dist1 && dist2 < min_dist &&
            turtlelib::check_line(
              x2, red_robot.get_config().x,
              max_x) > 0.0 && turtlelib::check_line(y2, red_robot.get_config().y, max_y) > 0.0)
          {
            min_dist = dist2;
          }
        }
        // // else if the discirimate is 0 there is one solution
        else if (determinate == 0.0) {
          double x = (-B) / (2 * A);
          double y = m * (x - red_robot.get_config().x) + red_robot.get_config().y;
          double dist =
            sqrt(
            (red_robot.get_config().x - x) * (red_robot.get_config().x - x) +
            (red_robot.get_config().y - y) * (red_robot.get_config().y - y));
          if (dist < min_dist &&
            turtlelib::check_line(
              x, red_robot.get_config().x,
              max_x) > 0.0 && turtlelib::check_line(y, red_robot.get_config().y, max_y) > 0.0)
          {
            min_dist = dist;
          }
        }
      }

      // check each wall for a collision
      // wall 1
      double x_wall1 = x_length / 2;
      double y_wall1 = m * (x_wall1 - red_robot.get_config().x) + red_robot.get_config().y;
      double dist_wall1 = turtlelib::get_dist(
        red_robot.get_config().x,
        red_robot.get_config().y, x_wall1, y_wall1);
      if (dist_wall1 < min_dist &&
        turtlelib::check_line(
          x_wall1, red_robot.get_config().x,
          max_x) > 0.0 && turtlelib::check_line(y_wall1, red_robot.get_config().y, max_y) > 0.0)
      {
        min_dist = dist_wall1;
      }

      // // wall 2
      double y_wall2 = y_length / 2;
      double x_wall2 = (y_wall2 - red_robot.get_config().y) / m + red_robot.get_config().x;
      double dist_wall2 = turtlelib::get_dist(
        red_robot.get_config().x,
        red_robot.get_config().y, x_wall2, y_wall2);
      if (dist_wall2 < min_dist &&
        turtlelib::check_line(
          x_wall2, red_robot.get_config().x,
          max_x) > 0.0 && turtlelib::check_line(y_wall2, red_robot.get_config().y, max_y) > 0.0)
      {
        min_dist = dist_wall2;
      }

      // wall 3
      double x_wall3 = -x_length / 2;
      double y_wall3 = m * (x_wall3 - red_robot.get_config().x) + red_robot.get_config().y;
      double dist_wall3 = turtlelib::get_dist(
        red_robot.get_config().x,
        red_robot.get_config().y, x_wall3, y_wall3);
      if (dist_wall3 < min_dist &&
        turtlelib::check_line(
          x_wall3, red_robot.get_config().x,
          max_x) > 0.0 && turtlelib::check_line(y_wall3, red_robot.get_config().y, max_y) > 0.0)
      {
        min_dist = dist_wall3;
      }

      // // wall 4
      double y_wall4 = -y_length / 2;
      double x_wall4 = (y_wall4 - red_robot.get_config().y) / m + red_robot.get_config().x;
      double dist_wall4 = turtlelib::get_dist(
        red_robot.get_config().x,
        red_robot.get_config().y, x_wall4, y_wall4);
      if (dist_wall4 < min_dist &&
        turtlelib::check_line(
          x_wall4, red_robot.get_config().x,
          max_x) > 0.0 && turtlelib::check_line(y_wall4, red_robot.get_config().y, max_y) > 0.0)
      {
        min_dist = dist_wall4;
      }

      // check if the min_dist is in range, if it is then push it back, if not push back 0.0
      if (min_dist < laser.range_max && min_dist > laser.range_min) {
        min_dist += laser_var(gen);
        laser.ranges.push_back(min_dist);
      } else {
        laser.ranges.push_back(0.0);
      }
    }

    // publish laser scan data
    laser_scan_publisher_->publish(laser);

    /// publish all of the obstacles
    obstacle_publisher_->publish(obs);
    wall_publisher_->publish(walls);
  }

  /// \brief service callback to reset the robot's pose
  /// \param - empty
  /// \param - empty
  void reset(
    std::shared_ptr<std_srvs::srv::Empty::Request>,
    std::shared_ptr<std_srvs::srv::Empty::Response>)
  {
    count_ = 0;
    x_0 = x_init;
    y_0 = y_init;
    theta_0 = theta_init;
    red_robot = turtlelib::DiffDrive{track, wheel_radius, {0.0, 0.0}, {theta_0, x_0, y_0}};
    return;
  }

  /// \brief service callback to teleport the robot
  /// \param request - x, y, theta values of the new location
  /// \param - empty
  void teleport(
    std::shared_ptr<nusim::srv::Teleport::Request> request,
    std::shared_ptr<nusim::srv::Teleport::Response>)
  {
    x_0 = request->x;
    y_0 = request->y;
    theta_0 = request->theta;
    double temp_phi_r = red_robot.get_wheels().phi_r;
    double temp_phi_l = red_robot.get_wheels().phi_l;
    red_robot =
      turtlelib::DiffDrive{track, wheel_radius, {temp_phi_r, temp_phi_l}, {theta_0, x_0, y_0}};
    return;
  }

  /// \brief initialize variables
  bool first;
  // variables declared in public
  double dt;
  double track;
  double wheel_radius, collision_radius;
  double radius;
  double motor_cmd_per_rad_sec;
  double encoder_ticks_per_rad;
  double x_length, y_length;
  double input_noise, slip_fraction;
  double basic_sensor_variance, max_range;
  double num_samples, laser_noise, min_laser_range, max_laser_range, angle_increment;
  double wall_thickness = 0.1;
  bool draw_only;
  std::vector<std::vector<double>> wall_pos;
  std::vector<double> wall_slope;
  //initilize to 0
  double phi_l_rad_s = 0.0, phi_r_rad_s = 0.0;
  double no_noise_phi_l_rad_s = 0.0, no_noise_phi_r_rad_s = 0.0;
  double x_0 = 0.0;
  double y_0 = 0.0;
  double theta_0 = 0.0;
  double x_init = 0.0;
  double y_init = 0.0;
  double theta_init = 0.0;
  std::vector<double> obs_x;
  std::vector<double> obs_y;
  turtlelib::Wheels delta_wheels = {0.0, 0.0};
  turtlelib::DiffDrive red_robot = {track, wheel_radius};
  // geometry_msgs::msg::PoseStamped poses;
  rclcpp::Time timestamp;
  nav_msgs::msg::Path path;
  visualization_msgs::msg::MarkerArray obs;
  visualization_msgs::msg::MarkerArray walls;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer2_;
  std_msgs::msg::UInt64::_data_type count_;
  std::normal_distribution<double> dist;
  std::uniform_real_distribution<double> uniform_dist;
  std::normal_distribution<double> laser_var;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fake_sensor_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr wall_publisher_;
  rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr wheel_cmd_subscription_;
  rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr sensor_data_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_publisher_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr service_;
  rclcpp::Service<nusim::srv::Teleport>::SharedPtr teleport_service_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  nuturtlebot_msgs::msg::SensorData sensor_readings;
  std::random_device rd;
  std::mt19937 gen;
};

/// \brief the main fucntion that spins the node
/// \param argc
/// \param argv
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  /// \brief try, catch to run the node if the length of the arrays match
  try {
    rclcpp::spin(std::make_shared<Nusim>());
  } catch (const std::exception &) {
    RCLCPP_ERROR(
      std::make_shared<Nusim>()->get_logger(), "NODE SHUTTING DOWN: INVALID ARRAY ENTRIES!!!");
    rclcpp::shutdown();
  }
  rclcpp::shutdown();
  return 0;
}
