#ifndef DIFF_DRIVE_INCLUDE_GUARD_HPP
#define DIFF_DRIVE_INCLUDE_GUARD_HPP

#include<iosfwd> // contains forward definitions for iostream objects
#include <cmath> 
#include"turtlelib/rigid2d.hpp"
namespace turtlelib
{
    /// \brief Wheel positions
    struct Wheels{
        /// \brief the right wheel position
        double phi_r;
        /// \brief the left wheel position
        double phi_l;
    };

    /// \brief The description of where a robot is (x, y, theta)
    struct Configuration{
        /// \brief the theta value
        double w;
        /// \brief the x coordinate
        double x;
        /// \brief the y coordinate
        double y;
    };

    /// \brief A description of a diff drive robot
    class DiffDrive{
        private:
            double track;
            double rad;
            Wheels phi;
            Configuration config;

        public:
            /// \brief create a DiffDrive object with no position or configuration
            /// \param track - track value
            /// \param rad - wheel radius
            DiffDrive(double track, double rad);

            /// \brief create a DiffDrive object
            /// \param track - track value
            /// \param rad - wheel radius
            /// \param phi - A wheels struct with the positon of the left and right wheels
            /// \param config - a Configuaration struct with the x, y, and w values of the robot
            DiffDrive(double track, double rad, Wheels phi, Configuration config);

            /// \brief get the track value
            /// \return the track value
            double get_track() const;

            /// \brief get the wheel radius
            /// \return the wheel radius
            double get_rad() const;

            /// \brief get the position of both the wheels
            /// \return the wheel positions
            Wheels get_wheels() const;

            /// \brief set new wheel positions for the right wheel
            /// \param new_phi_r - the new wheel positions
            void set_r_wheel(double new_phi_r) {phi.phi_r = new_phi_r;}

            /// \brief set new wheel positions the left wheel
            /// \param new_phi_l - the new wheel positions
            void set_l_wheel(double new_phi_l) {phi.phi_l = new_phi_l;}

            /// \brief get the configuration of the robot
            /// \return the robot config (w, x, y)
            Configuration get_config() const;

            /// \brief set new x position of the robot
            /// \param new_x - the new wheel positions
            void set_x(double new_x) {config.x = new_x;}

            /// \brief set new y position of the robot
            /// \param new_y - the new wheel positions
            void set_y(double new_y) {config.y = new_y;}

            /// \brief calculate the body twist
            /// \param phis - the new wheel positions
            Twist2D calc_bod_twist(Wheels phis) const;

            /// \brief calculate the position of the robot using forward kinematics
            /// \param phis - the new wheel positions
            DiffDrive calculate_FK(Wheels phis);

            /// \brief compute the wheel velocities to get to a given position with inverse kinematics
            /// \param twist - the desired body twist
            Wheels calculate_IK(Twist2D twist) const;
    };
};

#endif