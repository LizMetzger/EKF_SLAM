/// \file
/// \brief This node provides SLAM updates to create a better estimation of where the real robot is
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
///     \param /odom: the odom frame of the green robot
///     \param /green_path: publishes the path of the green robot
///     \param /green_sensor: publishes the markers identified by SLAM
///
/// SUBSCRIBES:
///     \param /joint_states: the joint staes of the robot
///     \param /Nusim_node/fake_sensor: the fake sensor data
///     \param /laser_landmarks: subscribes to the landmarks idetified by the laser scan

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <armadillo>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "turtlelib/diff_drive.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include <visualization_msgs/msg/marker_array.hpp>
#include "turtlelib/rigid2d.hpp"

#define MAX_OBS 5
#define DIST_THRESHOLD .5

/// \brief a node for keeping track of the simulated position of the robot
class Nuslam : public rclcpp::Node
{
public:
  Nuslam()
  : Node("Nuslam_Node")
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

    declare_parameter("fake_sensor", true);
    fake_sensor = get_parameter("fake_sensor").get_parameter_value().get<bool>();

    /// \brief create a subscription to joint_states
    /// topic: Odometry/joint_states (sensor_msgs::msg::JointState)
    joint_states_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&Nuslam::joint_states_callback, this, std::placeholders::_1));

    if (fake_sensor){
      /// \brief create a subscription the fake sensor data
      /// topic: /Nusim_node/fake_sensor (visualization_msgs::msg::MarkerArray)
      fake_sensor_subscription_ = create_subscription<visualization_msgs::msg::MarkerArray>(
        "/Nusim_node/fake_sensor", 10,
        std::bind(&Nuslam::fake_sensor_callback, this, std::placeholders::_1));
    }
    else{
      /// \brief create a subscription to the laser scan data
      /// topic: /laser_landmarks (visualization_msgs::msg::MarkerArray)
      laser_landmarks_subscription_ = create_subscription<visualization_msgs::msg::MarkerArray>(
        "/laser_landmarks", 10,
        std::bind(&Nuslam::laser_landmark_callback, this, std::placeholders::_1));
    }

    /// \brief create a publisher for the odometry
    /// topic: /odom (nav_msgs::msg::Odometry)
    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      "/odom", 10);

    /// \brief create a publisher for the path
    /// topic: /red/path (nav_msgs/msg/Path)
    path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "/green_path", 10);

    // Initialize the transform broadcaster
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Initialize the transform broadcaster
    mo_tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    /// \brief create a publisher for the obstacle markers as detected by slam
    /// topic: /green_sensor (visualization_msgs/msg/MarkerArray)
    green_sensor_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/green_sensor", 10);

    // initialize variables
    first = true;
    robot = turtlelib::DiffDrive{track, radius};
  }

private:
  /// @brief call back that takes in joint states to publish a transform of the
  /// new position of the robot
  /// @param msg - joint states message
  void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    count++;
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


    // publish the path
    if (count == 10) {
      path.poses.push_back(new_pose);
      path_publisher_->publish(path);
      count = 0;
    }
    return;
  }

  /// @brief do SLAM using the simulated marker data (from nusim.cpp)
  /// @param msg - a marker array with all of the markers beign published
  void fake_sensor_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
    // create a green marker array
    visualization_msgs::msg::MarkerArray green_obs;
    ///////// PREDICT /////////
    // calculate a new squiggle (xi)
    xi_prev(0) = turtlelib::normalize_angle(xi_prev(0));
    arma::vec q =
    {turtlelib::normalize_angle(robot.get_config().w), robot.get_config().x,
      robot.get_config().y};
    arma::vec delta_q = q - q_prev;
    delta_q(0) = turtlelib::normalize_angle(delta_q(0));
    arma::vec xi_minus = xi_prev + arma::join_cols(delta_q, zeros_2n);
    xi_minus(0) = turtlelib::normalize_angle(xi_minus(0));

    // calculate A
    arma::mat A = arma::mat(m_size + 3, m_size + 3, arma::fill::zeros);
    A(1, 0) = -delta_q(2);
    A(2, 0) = delta_q(1);
    arma::mat At = identity + A;

    // calculate a predicted sigma
    arma::mat sigma_minus = At * sigma_prev * At.t() + Q_bar;
    ///////// UPDATE /////////
    // for each object in the marker array
    for (int i = 0; i < (int)std::size(msg->markers); i++) {
      // if you can see the marker do an update
      if (msg->markers.at(i).action == visualization_msgs::msg::Marker::ADD) {
        // calculate r_j and phi_j
        double x_bar = msg->markers.at(i).pose.position.x;
        double y_bar = msg->markers.at(i).pose.position.y;
        double r_j = sqrt(x_bar * x_bar + y_bar * y_bar);
        double phi_j = turtlelib::normalize_angle(atan2(y_bar, x_bar));
        arma::vec z_i = {r_j, phi_j};

        // check if you've seen if before, if you haven't then initialize
        if (ids.at(i) == false) {
          // initialize the marker
          xi_minus(i * 2 + 3) = xi_minus(1) + r_j * std::cos(phi_j + turtlelib::normalize_angle(xi_minus(0)));
          xi_minus(i * 2 + 4) = xi_minus(2) + r_j * std::sin(phi_j + turtlelib::normalize_angle(xi_minus(0)));
          // set value to true to show its been seen before
          ids.at(i) = true;
        }

        /// DO AN UPDATE ///
        double delta_x = xi_minus(i * 2 + 3) - xi_minus(1);
        double delta_y = xi_minus(i * 2 + 4) - xi_minus(2);
        double d = delta_x * delta_x + delta_y * delta_y;
        double sqrt_d = sqrt(d);

        // find h_j (z theroetical)
        arma::vec h_j = arma::vec(2);
        h_j(0) = sqrt_d;
        h_j(1) =
          turtlelib::normalize_angle(
          atan2(
            delta_y,
            delta_x) - turtlelib::normalize_angle(xi_minus(0)));

        // find K_i
        // make H_i (2x2n-2+5)
        arma::mat H_i = arma::mat(2, m_size + 3, arma::fill::zeros);
        H_i(1, 0) = -1;
        H_i(0, 1) = -delta_x / sqrt_d;
        H_i(1, 1) = delta_y / d;
        H_i(0, 2) = -delta_y / sqrt_d;
        H_i(1, 2) = -delta_x / d;
        H_i(0, i * 2 + 3) = delta_x / sqrt_d;
        H_i(1, i * 2 + 3) = -delta_y / d;
        H_i(0, i * 2 + 4) = delta_y / sqrt_d;
        H_i(1, i * 2 + 4) = delta_x / d;
        arma::mat K_i = sigma_minus * H_i.t() * (H_i * sigma_minus * H_i.t() + R).i();
        arma::vec delta_z = z_i - h_j;
        delta_z(1) = turtlelib::normalize_angle(delta_z(1));
        xi_minus = xi_minus + K_i * (delta_z);
        xi_minus(0) = turtlelib::normalize_angle(xi_minus(0));
        sigma_minus = (identity - K_i * H_i) * sigma_minus;

        // add green markers according to SLAM
        visualization_msgs::msg::Marker green_marker;
        green_marker.header.frame_id = "map";
        green_marker.header.stamp = get_clock()->now();
        green_marker.action = visualization_msgs::msg::Marker::ADD;
        green_marker.type = visualization_msgs::msg::Marker::CYLINDER;
        green_marker.pose.position.x = xi_minus(i * 2 + 3);
        green_marker.pose.position.y = xi_minus(i * 2 + 4);
        green_marker.pose.position.z = 0.0;
        green_marker.pose.orientation.x = 0.0;
        green_marker.pose.orientation.y = 0.0;
        green_marker.pose.orientation.z = 0.0;
        green_marker.pose.orientation.w = 1.0;
        green_marker.id = i;
        green_marker.color.a = 1.0;
        green_marker.color.r = 0.0;
        green_marker.color.g = 1.0;
        green_marker.color.b = 0.0;
        green_marker.scale.x = radius;
        green_marker.scale.y = radius;
        green_marker.scale.z = 0.25;
        green_obs.markers.push_back(green_marker);
      }
    }
    // publish all of the green obstacles
    green_sensor_publisher_->publish(green_obs);

    // Find T_mo
    turtlelib::Transform2D T_or{{robot.get_config().x, robot.get_config().y},
      turtlelib::normalize_angle((robot.get_config().w))};
    turtlelib::Transform2D T_mr{{xi_minus(1), xi_minus(2)},
      turtlelib::normalize_angle(xi_minus(0))};
    turtlelib::Transform2D T_mo = T_mr * T_or.inv();

    tf2_quat2.setRPY(0.0, 0.0, T_mo.rotation());
    geometry_msgs::msg::TransformStamped mo;
    mo.header.stamp = get_clock()->now();
    mo.header.frame_id = "map";
    mo.child_frame_id = "green_odom";
    mo.transform.translation.x = T_mo.translation().x;
    mo.transform.translation.y = T_mo.translation().y;
    mo.transform.translation.z = 0.0;
    mo.transform.rotation.x = tf2_quat2.x();
    mo.transform.rotation.y = tf2_quat2.y();
    mo.transform.rotation.z = tf2_quat2.z();
    mo.transform.rotation.w = tf2_quat2.w();
    mo_tf_broadcaster_->sendTransform(mo);

    q_prev(0) = turtlelib::normalize_angle(xi_minus(0));
    q_prev(1) = xi_minus(1);
    q_prev(2) = xi_minus(2);
    sigma_prev = sigma_minus;
    xi_minus(0) = turtlelib::normalize_angle(xi_minus(0));
    xi_prev = xi_minus;
  }

  /// @brief do SLAM using the lidar information
  /// @param msg - a marker array with detected landmarks (from landmarks.cpp)
  void laser_landmark_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
    // create a green marker array
    visualization_msgs::msg::MarkerArray green_obs;
    ///////// PREDICT /////////
    // calculate a new squiggle (xi)
    xi_prev(0) = turtlelib::normalize_angle(xi_prev(0));
    arma::vec q =
    {turtlelib::normalize_angle(robot.get_config().w), robot.get_config().x,
      robot.get_config().y};
    arma::vec delta_q = q - q_prev;
    delta_q(0) = turtlelib::normalize_angle(delta_q(0));
    arma::vec xi_minus = xi_prev + arma::join_cols(delta_q, zeros_2n);
    xi_minus(0) = turtlelib::normalize_angle(xi_minus(0));

    // calculate A
    arma::mat A = arma::mat(m_size + 3, m_size + 3, arma::fill::zeros);
    A(1, 0) = -delta_q(2);
    A(2, 0) = delta_q(1);
    arma::mat At = identity + A;

    // calculate a predicted sigma
    arma::mat sigma_minus = At * sigma_prev * At.t() + Q_bar;
    ///////// UPDATE /////////
    // for each object in the marker array
    for (size_t i = 0; i < std::size(msg->markers); i++) {
      int marker_ind = 100;
      // calculate r_j and phi_j
      double x_bar = msg->markers.at(i).pose.position.x;
      double y_bar = msg->markers.at(i).pose.position.y;
      double r_j = sqrt(x_bar * x_bar + y_bar * y_bar);
      double phi_j = turtlelib::normalize_angle(atan2(y_bar, x_bar));
      arma::vec z_i = {r_j, phi_j};

      // if no landmarks have been seen then add this one and do SLAM with it 
      if (N_landmarks == 0){
        xi_minus(i * 2 + 3) = xi_minus(1) + r_j * std::cos(phi_j + turtlelib::normalize_angle(xi_minus(0)));
        xi_minus(i * 2 + 4) = xi_minus(2) + r_j * std::sin(phi_j + turtlelib::normalize_angle(xi_minus(0)));
        N_landmarks++;
        marker_ind = 0;
      }

      // else look at all the landmarks I've seen before to see if this is one we know
      else{
        auto N_landmarks_inc = N_landmarks + 1;
        // create a vec to store all of the mahalanobis distances
        arma::vec mahal_dist = arma::vec(N_landmarks_inc);
        // save xi_minus for if I decide this is fake
        arma::vec save_xi_minus = xi_minus;
        // initialize the marker as N+1
        // get x and y coords of new point
        save_xi_minus(N_landmarks * 2 + 3) = save_xi_minus(1) + r_j * std::cos(phi_j + turtlelib::normalize_angle(save_xi_minus(0)));
        save_xi_minus(N_landmarks * 2 + 4) = save_xi_minus(2) + r_j * std::sin(phi_j + turtlelib::normalize_angle(save_xi_minus(0)));

        // for each landmark find H_k, h_k, psi_k and d_k to get the mahalonobis distance for each one
        for (int j = 0; j < N_landmarks_inc; j ++){
          double delta_x = save_xi_minus(j * 2 + 3) - save_xi_minus(1);
          double delta_y = save_xi_minus(j * 2 + 4) - save_xi_minus(2);
          double d = delta_x * delta_x + delta_y * delta_y;
          double sqrt_d = sqrt(d);

          // make H_i (2x2n-2+5)
          arma::mat H_k = arma::mat(2, m_size + 3, arma::fill::zeros);
          H_k(1, 0) = -1;
          H_k(0, 1) = -delta_x / sqrt_d;
          H_k(1, 1) = delta_y / d;
          H_k(0, 2) = -delta_y / sqrt_d;
          H_k(1, 2) = -delta_x / d;
          H_k(0, j * 2 + 3) = delta_x / sqrt_d;
          H_k(1, j * 2 + 3) = -delta_y / d;
          H_k(0, j * 2 + 4) = delta_y / sqrt_d;
          H_k(1, j * 2 + 4) = delta_x / d;

          // Find psi_k
          arma::mat psi_k = H_k*sigma_minus*H_k.t() + R;

          // find h_j (z theroetical)
          arma::vec h_k = arma::vec(2);
          h_k(0) = sqrt_d;
          h_k(1) =
            turtlelib::normalize_angle(
            atan2(
              delta_y,
              delta_x) - turtlelib::normalize_angle(save_xi_minus(0)));
          
          // compute mahalanobis distance
          arma::vec delta_z_k = z_i - h_k;
          delta_z_k(1) = turtlelib::normalize_angle(delta_z_k(1));
          arma::mat temp_mah = delta_z_k.t()*psi_k.i()*delta_z_k;
          mahal_dist.at(j) = temp_mah.at(0);
        }

        // set the mahalonbis distance of landmark N+1 to be my threshold
        mahal_dist.at(N_landmarks) = DIST_THRESHOLD;
        double d_star = 999999;
        int d_star_ind = 0;
        // find d_star and d_star_ind
        for (int l = 0; l < N_landmarks_inc; l++){
          if (mahal_dist.at(l) < d_star){
            d_star = mahal_dist.at(l);
            d_star_ind = l;
            marker_ind = l;
          }
        }

        // if the index is the N+1 landmark then its new, update xi_minus 
        if (d_star_ind == N_landmarks){
          xi_minus = save_xi_minus;
          marker_ind = N_landmarks;
          // increment N_landmarks
          N_landmarks ++;
        }
    }

      /// DO AN UPDATE ///
      double delta_x = xi_minus(marker_ind * 2 + 3) - xi_minus(1);
      double delta_y = xi_minus(marker_ind * 2 + 4) - xi_minus(2);
      double d = delta_x * delta_x + delta_y * delta_y;
      double sqrt_d = sqrt(d);

      // find h_j (z theroetical)
      arma::vec h_j = arma::vec(2);
      h_j(0) = sqrt_d;
      h_j(1) =
        turtlelib::normalize_angle(
        atan2(
          delta_y,
          delta_x) - turtlelib::normalize_angle(xi_minus(0)));

      // find K_i
      // make H_i (2x2n-2+5)
      arma::mat H_i = arma::mat(2, m_size + 3, arma::fill::zeros);
      H_i(1, 0) = -1;
      H_i(0, 1) = -delta_x / sqrt_d;
      H_i(1, 1) = delta_y / d;
      H_i(0, 2) = -delta_y / sqrt_d;
      H_i(1, 2) = -delta_x / d;
      H_i(0, marker_ind * 2 + 3) = delta_x / sqrt_d;
      H_i(1, marker_ind * 2 + 3) = -delta_y / d;
      H_i(0, marker_ind * 2 + 4) = delta_y / sqrt_d;
      H_i(1, marker_ind * 2 + 4) = delta_x / d;
      arma::mat K_i = sigma_minus * H_i.t() * (H_i * sigma_minus * H_i.t() + R).i();

      arma::vec delta_z = z_i - h_j;
      delta_z(1) = turtlelib::normalize_angle(delta_z(1));
      xi_minus = xi_minus + K_i * (delta_z);
      sigma_minus = (identity - K_i * H_i) * sigma_minus;

      // add green markers according to SLAM
      visualization_msgs::msg::Marker green_marker;
      green_marker.header.frame_id = "map";
      green_marker.header.stamp = get_clock()->now();
      green_marker.action = visualization_msgs::msg::Marker::ADD;
      green_marker.type = visualization_msgs::msg::Marker::CYLINDER;
      green_marker.pose.position.x = xi_minus(marker_ind * 2 + 3);
      green_marker.pose.position.y = xi_minus(marker_ind * 2 + 4);
      green_marker.pose.position.z = 0.0;
      green_marker.pose.orientation.x = 0.0;
      green_marker.pose.orientation.y = 0.0;
      green_marker.pose.orientation.z = 0.0;
      green_marker.pose.orientation.w = 1.0;
      green_marker.id = marker_ind;
      green_marker.color.a = 1.0;
      green_marker.color.r = 0.0;
      green_marker.color.g = 1.0;
      green_marker.color.b = 0.0;
      green_marker.scale.x = radius;
      green_marker.scale.y = radius;
      green_marker.scale.z = 0.25;
      green_obs.markers.push_back(green_marker);
    }
    // publish all of the green obstacles
    green_sensor_publisher_->publish(green_obs);

    // Find T_mo
    turtlelib::Transform2D T_or{{robot.get_config().x, robot.get_config().y},
      turtlelib::normalize_angle((robot.get_config().w))};
    turtlelib::Transform2D T_mr{{xi_minus(1), xi_minus(2)},
      turtlelib::normalize_angle(xi_minus(0))};
    turtlelib::Transform2D T_mo = T_mr * T_or.inv();

    tf2_quat2.setRPY(0.0, 0.0, T_mo.rotation());
    geometry_msgs::msg::TransformStamped mo;
    mo.header.stamp = get_clock()->now();
    mo.header.frame_id = "map";
    mo.child_frame_id = "green_odom";
    mo.transform.translation.x = T_mo.translation().x;
    mo.transform.translation.y = T_mo.translation().y;
    mo.transform.translation.z = 0.0;
    mo.transform.rotation.x = tf2_quat2.x();
    mo.transform.rotation.y = tf2_quat2.y();
    mo.transform.rotation.z = tf2_quat2.z();
    mo.transform.rotation.w = tf2_quat2.w();
    mo_tf_broadcaster_->sendTransform(mo);

    q_prev(0) = turtlelib::normalize_angle(xi_minus(0));
    q_prev(1) = xi_minus(1);
    q_prev(2) = xi_minus(2);
    sigma_prev = sigma_minus;
    xi_minus(0) = turtlelib::normalize_angle(xi_minus(0));
    xi_prev = xi_minus;
  }

  /// \brief initialize variables
  double track, radius;
  bool first;
  int count;
  bool fake_sensor;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscription_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr fake_sensor_subscription_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr laser_landmarks_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr green_sensor_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> mo_tf_broadcaster_;
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
  tf2::Quaternion tf2_quat2;
  // max number of obstacles
  int n = MAX_OBS;
  int m_size = n * 2;
  arma::vec m_prev = arma::vec(m_size, arma::fill::zeros);
  arma::vec m = arma::vec(m_size, arma::fill::zeros);
  // id vector
  std::vector<bool> ids = std::vector<bool>(n, false);
  // initialize sigma matrices (error)
  arma::mat sigma_q = arma::mat(3, 3, arma::fill::zeros);
  arma::mat sigma_m = 99999 * arma::eye<arma::mat>(2 * n, 2 * n);
  arma::mat zeros_3x2n = arma::mat(3, m_size, arma::fill::zeros);
  arma::mat zeros_2nx3 = arma::mat(m_size, 3, arma::fill::zeros);
  arma::mat zeros_2nx2n = arma::mat(m_size, m_size, arma::fill::zeros);
  arma::vec zeros_2n = arma::vec(m_size, arma::fill::zeros);
  arma::mat sigma_top = std::move(arma::join_rows(sigma_q, zeros_3x2n));
  arma::mat sigma_bot = std::move(arma::join_rows(zeros_2nx3, sigma_m));
  arma::mat sigma_prev = std::move(arma::join_cols(sigma_top, sigma_bot));
  // initialize identity matrix (for A)
  arma::mat identity = arma::mat(3 + m_size, 3 + m_size, arma::fill::eye);
  // initilize squiggle matrix
  arma::vec q_prev = arma::vec(3, arma::fill::zeros);
  arma::vec xi_prev = arma::join_cols(q_prev, m_prev);
  double w_k = .05;
  double v_k = .01;
  arma::mat Q = w_k * arma::eye<arma::mat>(3, 3);
  arma::mat R = v_k * arma::eye<arma::mat>(2, 2);
  arma::mat Q_top = arma::join_rows(Q, zeros_3x2n);
  arma::mat zeros = arma::join_rows(zeros_2nx3, zeros_2nx2n);
  arma::mat Q_bar = arma::join_cols(Q_top, zeros);
  // number of landmarks seen'
  int N_landmarks = 0;
  bool update = false;
};

/// \brief the main fucntion that spins the node
/// \param argc
/// \param argv
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<Nuslam>());
  } catch (const std::exception &) {
    rclcpp::shutdown();
  }
  rclcpp::shutdown();
  return 0;
}
