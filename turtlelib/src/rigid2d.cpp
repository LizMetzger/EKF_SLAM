#include"turtlelib/rigid2d.hpp"
#include<iostream>

std::ostream & turtlelib::operator<<(std::ostream & os, const turtlelib::Vector2D & v){
    os << "[" << v.x << " " << v.y << "]";
    return os;
}

std::istream & turtlelib::operator>>(std::istream & is, turtlelib::Vector2D & v){
    char p = is.peek();
    if ((p == ']') | (p == '[')){
        is.get();
        is >> v.x;
        is >> v.y;
    }
    else{
        is >> v.x;
        is >> v.y;
    }
    is.clear();
    is.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    return is;
}

namespace turtlelib{
    Transform2D::Transform2D(): x_y{0.0, 0.0}, theta{0.0} {}

    Transform2D::Transform2D(Vector2D trans): x_y{trans.x, trans.y}, theta{0.0} {}

    Transform2D::Transform2D(double radians): x_y{0.0, 0.0}, theta{radians} {}

    Transform2D::Transform2D(Vector2D trans, double radians): x_y{trans.x, trans.y}, theta{radians} {}

    Vector2D Transform2D::operator()(Vector2D v) const{
        double new_x = std::cos(theta)*v.x - std::sin(theta)*v.y + x_y.x;
        double new_y = std::sin(theta)*v.x + std::cos(theta)*v.y + x_y.y;
        return {new_x, new_y};
    }

    Transform2D Transform2D::inv() const{
        double new_theta = -theta;
        double new_x = -x_y.x*std::cos(theta) - x_y.y*std::sin(theta);
        double new_y = -x_y.y*std::cos(theta) + x_y.x*std::sin(theta);
        return Transform2D({new_x, new_y}, new_theta);
    }

    Transform2D & Transform2D::operator*=(const Transform2D & rhs){
        x_y.x = std::cos(this->theta)*rhs.x_y.x - std::sin(this->theta)*rhs.x_y.y + this->x_y.x;
        x_y.y = std::sin(this->theta)*rhs.x_y.x + std::cos(this->theta)*rhs.x_y.y + this->x_y.y;
        theta = this -> theta + rhs.theta;
        if (std::abs(this->theta) > 2.0*PI){
            this -> theta = std::fmod(this->theta, 2*PI);
        }
        return *this;
    }

    Vector2D Transform2D::translation() const{
        return {x_y.x, x_y.y};
    }

    double Transform2D::rotation() const{
        return theta;
    }

    Twist2D Transform2D::operator()(Twist2D twist) const{
        double temp_ang =  twist.angular;
        double temp_x = x_y.y*twist.angular + std::cos(theta)*twist.translate.x - std::sin(theta)*twist.translate.y;
        double temp_y = -x_y.x*twist.angular + std::sin(theta)*twist.translate.x + std::cos(theta)*twist.translate.y;
        return {temp_ang,{temp_x, temp_y}};
    }

    std::ostream & operator<<(std::ostream & os, const Transform2D & tf){
        os << "deg: " << rad2deg(tf.theta) << " x: " << tf.x_y.x << " y: " << tf.x_y.y;
        return os;
    }

    std::istream & operator>>(std::istream & is, Transform2D & tf){
        double temp_x = 0.0;
        double temp_y = 0.0;
        double temp_rad = 0.0;
        std::string str1, str2, str3;
        char p = is.peek();
        if (p == 'd'){
            is >> str1 >> temp_rad >> str2 >> temp_x >> str3 >> temp_y;
        }
        else{
            is >> temp_rad >> temp_x >> temp_y;
        }
        Vector2D temp_vec = {temp_x, temp_y};
        Transform2D temp_trans = {temp_vec, deg2rad(temp_rad)};
        tf = temp_trans;
        is.clear();
        is.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        return is;
    }

    Transform2D operator*(Transform2D lhs, const Transform2D & rhs){
        lhs*=rhs;
        return lhs;
    }

    // Twist functions
    Twist2D::Twist2D(): angular{0.0}, translate{0.0, 0.0} {}

    Twist2D::Twist2D(Vector2D trans): angular{0.0}, translate{trans.x, trans.y} {}

    Twist2D::Twist2D(double radians): angular{radians}, translate{0.0, 0.0} {}

    Twist2D::Twist2D(double ang, Vector2D trans): angular{ang}, translate{trans.x, trans.y} {}

    std::ostream & operator<<(std::ostream & os, const Twist2D & tf){
        os << "[" << tf.angular << " " << tf.translate.x << " " << tf.translate.y << "]";
        return os;
    }

    std::istream & operator>>(std::istream & is, Twist2D & tf){
        double temp_x = 0;
        double temp_y = 0;
        double temp_ang = 0;
        std::string str1, str2, str3;
        char p = is.peek();
        if (p == 'd'){
            is >> str1 >> temp_ang >> str2 >> temp_x >> str3 >> temp_y;
        }
        else{
            is >> temp_ang >> temp_x >> temp_y;
        }
        Vector2D temp_vec = {temp_x, temp_y};
        Twist2D temp_trans = {temp_ang, temp_vec};
        tf = temp_trans;
        is.clear();
        is.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        return is;
    }

    // normalize a 2D vector
    Vector2D normalize(Vector2D vec){
        double mag = sqrt(pow(vec.x, 2) + pow(vec.y, 2));
        return {vec.x/mag, vec.y/mag};
    }

    // normalize an angle
    double normalize_angle(double rad){
        while (rad <= -PI || rad > PI){
            if (rad <= (-PI)){
                rad = rad + 2*PI;
            }
            else if (rad > PI){
                rad = rad - 2*PI;
            }
        }
        return rad;
    }


    // Vector2D funcitons
    Vector2D & Vector2D::operator+=(const Vector2D & rhs){
        this -> x = this -> x + rhs.x;
        this -> y = this -> y + rhs.y;
        return *this;
    }

    Vector2D operator+(Vector2D & lhs, const Vector2D & rhs){
        Vector2D ans = lhs+=rhs;
        return ans; 
    }

    Vector2D & Vector2D::operator-=(const Vector2D & rhs){
        this -> x = this -> x - rhs.x;
        this -> y = this -> y - rhs.y;
        return *this;
    }

    Vector2D operator-(Vector2D & lhs, const Vector2D & rhs){
        Vector2D ans = lhs-=rhs;
        return ans;
    }

    Vector2D & Vector2D::operator*=(double scalar){
        this -> x = this -> x * scalar;
        this -> y = this -> y * scalar;
        return *this;
    }

    Vector2D operator*(Vector2D & rhs, double scalar){
        Vector2D ans = rhs*=scalar;
        return ans;
    }

    Vector2D operator*(double scalar, const Vector2D & rhs){
        Vector2D ans = {rhs.x*scalar, rhs.y*scalar};
        return ans;
    }

    double dot(const Vector2D & lhs, const Vector2D & rhs){
        double ans = lhs.x*rhs.x + lhs.y*rhs.y;
        return ans;
    }

    double magnitude(Vector2D vec){
        double ans = sqrt(pow(vec.x, 2) + pow(vec.y, 2));
        return ans;
    }

    double angle(const Vector2D lhs, const Vector2D rhs){
        double ans = acos((dot(lhs,rhs))/(magnitude(lhs)*magnitude(rhs)));
        return ans;
    }

    // integrate twist
    Transform2D integrate_twist(Twist2D twist){
        if (twist.getAng() == 0.0){
            double x = twist.getTran().x;
            double y = twist.getTran().y;
            Transform2D Tbb = {{x, y}, 0.0};
            return Tbb;
        }
        else{
            double x = twist.getTran().y/twist.getAng();
            double y = -twist.getTran().x/twist.getAng();

            Transform2D Tsb = {{x, y}, 0.0};

            Transform2D Tbs = Tsb.inv();

            Transform2D Tss = {{0.0,0.0}, twist.getAng()};

            Transform2D Tbb = Tbs*Tss;
            Tbb *= Tsb;
            return Tbb;
        }
    }
}
