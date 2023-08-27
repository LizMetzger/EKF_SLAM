#include"turtlelib/rigid2d.hpp"
#include<iostream>

int main(){
    std::cout << "Enter transform T_{a,b}: " << std::endl;
    turtlelib::Transform2D T_ab;
    std::cin >> T_ab;

    std::cout <<  "Enter transform T_{b,c}: " << std::endl;
    turtlelib::Transform2D T_bc;
    std::cin >> T_bc;

    turtlelib::Transform2D T_ba = T_ab.inv();
    turtlelib::Transform2D T_cb = T_bc.inv();
    turtlelib::Transform2D T_ac = T_ab*T_bc;
    turtlelib::Transform2D T_ca = T_ac.inv();

    std::cout << "T_{a,b}: " << T_ab << "\n";
    std::cout << "T_{b,a}: " << T_ba << "\n";
    std::cout << "T_{b,c}: " << T_bc << "\n";
    std::cout << "T_{c,b}: " << T_cb << "\n";
    std::cout << "T_{a,c}: " << T_ac << "\n";
    std::cout << "T_{c,a}: " << T_ca << "\n";

    std::cout << "Enter vector v_b: " << std::endl;
    turtlelib::Vector2D v_b;
    std::cin >> v_b;

    turtlelib::Vector2D v_bhat = normalize(v_b);
    turtlelib::Vector2D v_a = T_ab(v_b);
    turtlelib::Vector2D v_c = T_cb(v_b);

    std::cout << "v_bhat: " << v_bhat << "\n";
    std::cout << "v_a: " << v_a << "\n";
    std::cout << "v_b: " << v_b << "\n";
    std::cout << "v_c: " << v_c << "\n";

    std::cout << "Enter twist V_b: " << std::endl;
    turtlelib::Twist2D V_b;
    std::cin >> V_b;

    turtlelib::Twist2D V_a = T_ab(V_b);
    turtlelib::Twist2D V_c = T_cb(V_b);

    std::cout << "V_a: " << V_a << "\n";
    std::cout << "V_b: " << V_b << "\n";
    std::cout << "V_c: " << V_c << "\n";
}