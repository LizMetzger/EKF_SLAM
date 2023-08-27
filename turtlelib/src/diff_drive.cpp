#include"turtlelib/diff_drive.hpp"

#include<iostream>
#include<stdexcept>

namespace turtlelib{
    DiffDrive::DiffDrive(double track, double rad): track{track}, rad{rad}, phi{0.0, 0.0}, config{0.0, 0.0, 0.0} {}

    DiffDrive::DiffDrive(double track, double rad, Wheels phi, Configuration config): track{track}, rad{rad}, phi{phi}, config{config} {}

    double DiffDrive::get_track() const{
        return track;
    }

    double DiffDrive::get_rad() const{
        return rad;
    }

    Wheels DiffDrive::get_wheels() const{
        return {phi.phi_r, phi.phi_l};
    }

    Configuration DiffDrive::get_config() const{
        return {config.w, config.x, config.y};
    }

    Twist2D DiffDrive::calc_bod_twist(Wheels phis) const{
        // these formulas are from equations 5 and 6 of my notes
        return {(rad/track*(phis.phi_r - phis.phi_l)), {(rad/2.0*(phis.phi_r + phis.phi_l)), 0}};
    }

    DiffDrive DiffDrive::calculate_FK(Wheels phis){
        // get the body twist
        Twist2D temp_twist = calc_bod_twist(phis);
        // integrate
        Transform2D Tbb = integrate_twist(temp_twist);
        const auto world_x = Tbb.translation().x*std::cos(config.w) - Tbb.translation().y*std::sin(config.w);             // these formulas are from equations 7 and 8 of my notes
        const auto world_y = Tbb.translation().x*std::sin(config.w) + Tbb.translation().y*std::cos(config.w);
        config.x += world_x;
        config.y += world_y;
        phi.phi_r += phis.phi_r;
        phi.phi_l += phis.phi_l;
        config.w += Tbb.rotation();
        return *this;
    }

    Wheels DiffDrive::calculate_IK(Twist2D twist) const{
        if (turtlelib::almost_equal(twist.getTran().y, 0.0, .000001)){
            const auto temp_phi_l = (twist.getTran().x/rad) - (track/2)*(twist.getAng()/rad);      // these formulas are from equation 4 of my notes
            const auto temp_phi_r = (twist.getTran().x/rad) + (track/2)*(twist.getAng()/rad);
            return {temp_phi_r, temp_phi_l};
        }
        else{
            throw(std::logic_error("Desired configuration cannot have a non-zero y-component"));
        }
    }
}
