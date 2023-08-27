/// \file
/// \brief This node simulates purple markers in rviz to represent detected circles and potential landmarks
///
/// PUBLISHES:
///     \param /laser_landmarks: publishes landmarks detected by the clustering and circle finding
///
/// SUBSCRIBES:
///     \param /red/laser_scan: subscribes to the laser scan data from the red robot

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "turtlelib/diff_drive.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include "turtlelib/rigid2d.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "turtlelib/make_circles.hpp"

#define CLUSTER_THRESHOLD .3
#define CIRCLE_HIGH_THRESHOLD .1
#define CIRCLE_LOW_THRESHOLD .01

/// \brief a that generates landmarks from unprocessed sensor data
class Landmarks : public rclcpp::Node
{
public:
  Landmarks()
  : Node("Landmarks_node")
  {
    /// \brief the laser scan data from the red robot
    /// topic: /red/laser_scan (sensor_msgs::msg::LaserScan)
    laser_scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/red/laser_scan", 10,
      std::bind(&Landmarks::laser_scan_callback, this, std::placeholders::_1));

    /// \brief the markers extrapolated from laser scan data
    /// topic: /laser_landmarks (visualization_msgs/msg/MarkerArray)
    laser_landmarks_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/laser_landmarks", 10);
  }

private:
  /// @brief takes in laser scan data and performs clustering and circle identification to create circles to publish as potential obstacles
  /// @param msg laser scan data from the red robot
  void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    /////// CLUSTERING //////
    // cluster_x = {Point1, Point2, Point3}
    // clusters = {{cluster_1}, {cluster_2}, {cluster_3}}
    std::vector<std::vector<turtlelib::Point>> clusters;
    std::vector<turtlelib::Point> cluster;
    //// Make initital clusters ////
    for (int i = 0; i < (int)std::size(msg->ranges); i++){
        turtlelib::Point coords;
        // only do all of this is the distance value isn't 0
        if (msg->ranges.at(i)){
            // convert the distance into relative coordinates (X, y)
            double theta = i * 0.01745329;
            coords.x = std::cos(theta)*msg->ranges.at(i);
            coords.y = std::sin(theta)*msg->ranges.at(i);

            // if there are no points in the cluster
            if (size(cluster) == 0){
                cluster.push_back(coords);
            }

            // else check normally
            else{
                // find the distance between the current point and the last point in the cluster
                double point_dist = turtlelib::get_dist(coords.x, coords.y, cluster.back().x, cluster.back().y);

                // if the distance is less than my cluster threshold then add it to the cluster
                if (point_dist < CLUSTER_THRESHOLD){
                    cluster.push_back(coords);
                }
                // otherwise
                else{
                    //add the current cluster to clusters
                    clusters.push_back(cluster);
                    // clear cluster
                    cluster.clear();
                    // add the current point to cluster (start a new cluster)
                    cluster.push_back(coords);
                }
            }
        }
    }

    //// Handle wrap around ////
    // check if the first and last cluster are the same
    double first_last_dist = turtlelib::get_dist(clusters.at(0).at(0).x, clusters.at(0).at(0).y, clusters.back().back().x, clusters.back().back().y);
    if (first_last_dist < CLUSTER_THRESHOLD){
        // concatenate the first and last vectors
        std::vector<turtlelib::Point> cat_cluster;
        clusters.at(0).insert(clusters.at(0).end(), clusters.back().begin(), clusters.back().end());
        cat_cluster = clusters.at(0);
        // remove them from the list of clusters
        clusters.erase(clusters.begin());
        clusters.erase(clusters.end() -1);
        // add the concaenated cluster to clusters
        clusters.push_back(cat_cluster);
    }

    //// Filter by size ////
    // remove any cluster that are less than 4 points (Change 4 to a bigger number if circles go bad)
    for (int j = 0; j < static_cast<int32_t>(clusters.size()); j ++){
        if (clusters.at(j).size() <= 4){
            clusters.erase(clusters.begin() + j);
            j--;
        }
    }

    //////// FIND CIRCLES ///////
    std::vector<turtlelib::Circle> circles;
    // call cluster fit algorithim for each cluster
    for (int k = 0; k < static_cast<int32_t>(clusters.size()); k ++){
        // only keep circles that are within the expected size range
        turtlelib::Circle temp_circle = find_circles(clusters.at(k));
        if (temp_circle.radius < CIRCLE_HIGH_THRESHOLD && temp_circle.radius > CIRCLE_LOW_THRESHOLD){
            circles.push_back(temp_circle);
        }
    }

    ////// PUBLISH CIRCLES //////
    visualization_msgs::msg::MarkerArray real_circles;
    // the offset is the difference betweem the red robot and the green robot
    // offset = 
    for (int l = 0; l < static_cast<int32_t>(circles.size()); l ++){
        visualization_msgs::msg::Marker new_circle;
        new_circle.header.frame_id = "red/base_scan";
        new_circle.header.stamp = get_clock()->now();
        new_circle.action = visualization_msgs::msg::Marker::ADD;
        new_circle.type = visualization_msgs::msg::Marker::CYLINDER;
        new_circle.pose.position.x = circles.at(l).center.x;
        new_circle.pose.position.y = circles.at(l).center.y;
        new_circle.pose.position.z = 0.0;
        new_circle.pose.orientation.x = 0.0;
        new_circle.pose.orientation.y = 0.0;
        new_circle.pose.orientation.z = 0.0;
        new_circle.pose.orientation.w = 1.0;
        new_circle.id = l;
        new_circle.color.a = 1.0;
        new_circle.color.r = 0.62;
        new_circle.color.g = 0.125;
        new_circle.color.b = 0.945;
        new_circle.scale.x = circles.at(l).radius;
        new_circle.scale.y = circles.at(l).radius;
        new_circle.scale.z = 0.25;
        new_circle.lifetime = rclcpp::Duration::from_seconds(0.2);
        real_circles.markers.push_back(new_circle);
    }
    laser_landmarks_publisher_->publish(real_circles);

    return;
  }

  /// \brief initialize variables
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr laser_landmarks_publisher_;
  // auto x_offset 0.0;
  // // x_offset = 0.0;
  // auto y_offset = 0.0;
  // y_offset = 0.0;
};

/// \brief the main fucntion that spins the node
/// \param argc
/// \param argv
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<Landmarks>());
  } catch (const std::exception &) {
    rclcpp::shutdown();
  }
  rclcpp::shutdown();
  return 0;
}
