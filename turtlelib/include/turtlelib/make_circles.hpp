#ifndef MAKE_CIRCLES_INCLUDE_GUARD_HPP
#define MAKE_CIRCLES_INCLUDE_GUARD_HPP

#include <iosfwd> // contains forward definitions for iostream objects
#include <cmath> 
#include <vector>
#include <armadillo>
#include <iostream>

namespace turtlelib
{
    /// \brief A cartesian point
    struct Point{
        /// \brief the x coord
        double x;
        /// \brief the y coord
        double y;
    };

    /// \brief The center and radius of a circle
    struct Circle{
        /// \brief the x and y coords of the center
        Point center;
        /// \brief the radius
        double radius;
    };

    /// @brief find the distance between two points
    /// @param x1 - x coord of point 1
    /// @param y1 - y coord of point 1
    /// @param x2 - x coord of point 2
    /// @param y2 - y coord of point 2
    /// @return
    double get_dist(double x1, double y1, double x2, double y2);

    /// @brief function to check if a point is between the robot and the max value
    /// @param new_point - the point to check (x or y value)
    /// @param robot_point - the current position of the robot (x or y value)
    /// @param max - the max coord (x or y value)
    /// @return
    double check_line(double new_point, double robot_point, double max);

    /// @brief 
    /// @param cluster 
    /// @return 
    Point find_centroid(std::vector<Point> cluster);

    /// @brief 
    /// @param shifted_cluster 
    /// @return 
    double find_z_bar(std::vector<Point> shifted_cluster);

    arma::mat find_Z_mat(std::vector<Point> shifted_cluster);

    Circle find_circles(std::vector<Point> cluster);
};

#endif