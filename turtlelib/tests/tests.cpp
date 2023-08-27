#include"turtlelib/rigid2d.hpp"
#include"turtlelib/diff_drive.hpp"
#include"turtlelib/make_circles.hpp"
#include<catch2/catch_test_macros.hpp>
#include<catch2/matchers/catch_matchers_floating_point.hpp>
#include<sstream>
#include<iostream>
#include<vector>

namespace turtlelib{
    TEST_CASE("Rotation()","[transform]"){ // Marno, Nel
    double test_rot = 30.0;
    Transform2D T_test{test_rot};
    REQUIRE(T_test.rotation() == test_rot);
    }

    TEST_CASE( "Translation", "[transform]" ){ // Ava, Zahedi
    turtlelib::Vector2D vec;
    vec.x = 1.0;
    vec.y = 3.4;
    turtlelib::Transform2D tf = turtlelib::Transform2D(vec);
    REQUIRE(turtlelib::almost_equal(tf.translation().x, 1.0, 0.00001) );
    REQUIRE(turtlelib::almost_equal(tf.translation().y, 3.4, 0.00001) );
    }


    TEST_CASE("operator() for a vector", "[transform]"){ // Liz, Metzger
    Vector2D answer = {1.3397643622, 0.910905899};
    Vector2D test_vec = {-1.0, 0.5};
    Vector2D trans_vec = {1, 0};
    Transform2D test_trans = {trans_vec, 30};
    Vector2D result = test_trans(test_vec);
    REQUIRE(almost_equal(result.x, answer.x));
    REQUIRE(almost_equal(result.y, answer.y));
    }

    TEST_CASE("operator() for a twist", "[transform]"){ // Liz, Metzger
    // twist to multiply
    Vector2D twist_vec = {-1.0, 0.5};
    Twist2D test_twist = {-2.0, twist_vec};
    // transform to multiply
    Vector2D trans_vec = {1, 0};
    Transform2D test_trans = {trans_vec, 30};
    Twist2D result = test_trans(test_twist);
    REQUIRE(almost_equal(result.getAng(), -2));
    REQUIRE(almost_equal(result.getTran().x, 0.339764));
    REQUIRE(almost_equal(result.getTran().y, 3.06516));
    }

    TEST_CASE("inv() for Transform2D", "[transform]"){ // Liz, Metzger
    Vector2D trans_vec = {3, 4};
    Transform2D test_trans = {trans_vec, 0.523599};
    Transform2D test_inv = test_trans.inv();
    REQUIRE(almost_equal(test_inv.translation().x, -4.598080762));
    REQUIRE(almost_equal(test_inv.translation().y, -1.964101615));
    REQUIRE(almost_equal(test_inv.rotation(), -0.523599));
    }

    TEST_CASE("operator *= for Transform2D", "[transform]"){ //Liz Metzger
    Vector2D trans_vec = {3, 4};
    Transform2D test_trans = {trans_vec, 0.523599};
    test_trans *= test_trans;
    std::cout << test_trans.translation().x << " " << test_trans.translation().y << " " << test_trans.rotation();
    REQUIRE(almost_equal(test_trans.translation().x, 3.59808));
    REQUIRE(almost_equal(test_trans.translation().y, 9.26314));
    REQUIRE(almost_equal(test_trans.rotation(), 1.0472));
    }

    TEST_CASE("Output stream operator <<", "[transform]"){    //Megan, Sindelar
    std::stringstream ss;                                   //create an object of stringstream
    std::streambuf* old_cout_streambuf = std::cout.rdbuf();     //pointer that holds the output
    std::cout.rdbuf(ss.rdbuf());        //redirect output stream to a stringstream
    turtlelib::Transform2D T_m = {{4, 3}, 0};
    std::cout << T_m << std::endl;
    std::cout.rdbuf(old_cout_streambuf);    //restore the original stream buffer
    REQUIRE(ss.str() == "deg: 0 x: 4 y: 3\n");
    }


    TEST_CASE("Operator>> for Transform2D", "[transform]") { // Dilan Wijesinghe
    std::stringstream input;
    turtlelib::Transform2D TfTest;
    input << "90 2 3";
    input >> TfTest;
    REQUIRE(turtlelib::almost_equal(TfTest.rotation(), turtlelib::deg2rad(90), 1.0e-5) );
    REQUIRE(turtlelib::almost_equal(TfTest.translation().x, 2.0, 1.0e-5) );
    REQUIRE(turtlelib::almost_equal(TfTest.translation().y, 3.0, 1.0e-5) );
    }

    TEST_CASE("normalize_angle()", "[transform]" ){ // Liz, Metzger
    double pi = normalize_angle(turtlelib::PI);
    double npi = normalize_angle(-turtlelib::PI);
    double zero = normalize_angle(0);
    double pi_four = normalize_angle(-turtlelib::PI/4);
    double pi_two = normalize_angle((3*turtlelib::PI)/2);
    double five_pi_two = normalize_angle((-5*turtlelib::PI)/2);
    REQUIRE(turtlelib::almost_equal(pi, turtlelib::PI, 0.00001));
    REQUIRE(turtlelib::almost_equal(npi, turtlelib::PI, 0.00001));
    REQUIRE(turtlelib::almost_equal(zero, 0.0, 0.00001));
    REQUIRE(turtlelib::almost_equal(pi_four, -turtlelib::PI/4, 0.00001));
    REQUIRE(turtlelib::almost_equal(pi_two, ((3*turtlelib::PI)/2) - 2*turtlelib::PI, 0.00001));
    REQUIRE(turtlelib::almost_equal(five_pi_two, ((-5*turtlelib::PI)/2) + 2*turtlelib::PI, 0.00001));
    }

    TEST_CASE("Operator += for Vector2D", "[transform]" ){ // Liz, Metzger
    turtlelib::Vector2D vec1 = {1.0, 2.0};
    turtlelib::Vector2D vec2 = {3.0, 4.0};
    vec1 += vec2;
    REQUIRE(turtlelib::almost_equal(vec1.x, 4.0, 0.00001));
    REQUIRE(turtlelib::almost_equal(vec1.y, 6.0, 0.00001));
    }

    TEST_CASE("Operator + for Vector2D", "[transform]" ){ // Liz, Metzger
    turtlelib::Vector2D vec1 = {1.0, 2.0};
    turtlelib::Vector2D vec2 = {3.0, 4.0};
    turtlelib::Vector2D vec3 = vec1 + vec2;
    REQUIRE(turtlelib::almost_equal(vec3.x, 4.0, 0.00001));
    REQUIRE(turtlelib::almost_equal(vec3.y, 6.0, 0.00001));
    }

    TEST_CASE("Operator -= for Vector2D", "[transform]" ){ // Liz, Metzger
    turtlelib::Vector2D vec1 = {1.0, 2.0};
    turtlelib::Vector2D vec2 = {3.0, 4.0};
    vec1 -= vec2;
    REQUIRE(turtlelib::almost_equal(vec1.x, -2.0, 0.00001));
    REQUIRE(turtlelib::almost_equal(vec1.y, -2.0, 0.00001));
    }

    TEST_CASE("Operator - for Vector2D", "[transform]" ){ // Liz, Metzger
    turtlelib::Vector2D vec1 = {1.0, 2.0};
    turtlelib::Vector2D vec2 = {3.0, 4.0};
    turtlelib::Vector2D vec3 = vec1 - vec2;
    REQUIRE(turtlelib::almost_equal(vec3.x, -2.0, 0.00001));
    REQUIRE(turtlelib::almost_equal(vec3.y, -2.0, 0.00001));
    }

    TEST_CASE("Operator *= for Vector2D", "[transform]" ){ // Liz, Metzger
    turtlelib::Vector2D vec1 = {1.0, 2.0};
    vec1*=2;
    REQUIRE(turtlelib::almost_equal(vec1.x, 2.0, 0.00001));
    REQUIRE(turtlelib::almost_equal(vec1.y, 4.0, 0.00001));
    }

    TEST_CASE("Operator * for Vector2D", "[transform]" ){ // Liz, Metzger
    turtlelib::Vector2D vec1 = {1.0, 2.0};
    turtlelib::Vector2D vec2 = {3.0, 4.0};
    turtlelib::Vector2D vec3 = 3*vec1;
    turtlelib::Vector2D vec4 = vec2*4;
    REQUIRE(turtlelib::almost_equal(vec3.x, 3.0, 0.00001));
    REQUIRE(turtlelib::almost_equal(vec3.y, 6.0, 0.00001));
    REQUIRE(turtlelib::almost_equal(vec4.x, 12.0, 0.00001));
    REQUIRE(turtlelib::almost_equal(vec4.y, 16.0, 0.00001));
    }

    TEST_CASE("dot() for Vector2D", "[transform]" ){ // Liz, Metzger
    turtlelib::Vector2D vec1 = {1.0, 2.0};
    turtlelib::Vector2D vec2 = {3.0, 4.0};
    double ans = dot(vec1, vec2);
    REQUIRE(turtlelib::almost_equal(ans, 11.0, 0.00001));
    }

    TEST_CASE("magnitude() for Vector2D", "[transform]" ){ // Liz, Metzger
    turtlelib::Vector2D vec1 = {2.0, 3.0};
    double ans = magnitude(vec1);
    REQUIRE(turtlelib::almost_equal(ans, 3.6055512, 0.00001));
    }

    TEST_CASE("angle() for Vector2D", "[transform]" ){ // Liz, Metzger
    turtlelib::Vector2D vec1 = {0.0, 1.0};
    turtlelib::Vector2D vec2 = {1.0, 0.0};
    turtlelib::Vector2D vec3 = {3.0, 4.0};
    double ans = angle(vec1, vec2);
    double ans2 = angle(vec2, vec3);
    REQUIRE(turtlelib::almost_equal(ans, 1.5708, 0.00001));
    REQUIRE(turtlelib::almost_equal(ans2, 0.927295, 0.00001));
    }

    TEST_CASE("integrate_twist() for Transform2D", "[transform]" ){ // Liz, Metzger
    turtlelib::Twist2D twist = {-1.24, {-2.15, -2.92}};
    turtlelib::Twist2D twist_rot = {0.0, {-2.15, -2.92}};
    turtlelib::Twist2D twist_trans = {-1.24, {0.0, 0.0}};
    turtlelib::Transform2D ans;
    turtlelib::Transform2D ans_no_rot;
    turtlelib::Transform2D ans_no_trans;
    ans = integrate_twist(twist);
    ans_no_rot = integrate_twist(twist_rot);
    ans_no_trans = integrate_twist(twist_trans);
    REQUIRE(turtlelib::almost_equal(ans.translation().x, -3.229863264722, 0.00001));
    REQUIRE(turtlelib::almost_equal(ans.translation().y, -1.05645265317421, 0.00001));
    REQUIRE(turtlelib::almost_equal(ans.rotation(), -1.24, 0.00001));
    REQUIRE(turtlelib::almost_equal(ans_no_rot.translation().x, -2.15, 0.00001));
    REQUIRE(turtlelib::almost_equal(ans_no_rot.translation().y, -2.92, 0.00001));
    REQUIRE(turtlelib::almost_equal(ans_no_rot.rotation(), 0.0, 0.00001));
    REQUIRE(turtlelib::almost_equal(ans_no_trans.translation().x, 0.0, 0.00001));
    REQUIRE(turtlelib::almost_equal(ans_no_trans.translation().y, 0.0, 0.00001));
    REQUIRE(turtlelib::almost_equal(ans_no_trans.rotation(), -1.24, 0.00001));
    }

    TEST_CASE("Forward movement FK and IK", "[transform]" ){ // Liz, Metzger
    turtlelib::DiffDrive robot = {1.0, 1.0, {0.0, 0.0}, {0.0, 0.0, 0.0}};
    turtlelib::Wheels new_phis = {1.0, 1.0};
    turtlelib::Twist2D new_config = {0.0, {1.0, 0.0}};
    robot.calculate_FK(new_phis);
    REQUIRE_THAT(robot.get_track(), Catch::Matchers::WithinAbs(1.0, 0.00001));
    REQUIRE_THAT(robot.get_rad(), Catch::Matchers::WithinAbs(1.0, 0.00001));
    REQUIRE_THAT(robot.get_wheels().phi_l, Catch::Matchers::WithinAbs(1.0, 1e-5));
    REQUIRE_THAT(robot.get_wheels().phi_r, Catch::Matchers::WithinAbs(1.0, 1e-5));
    REQUIRE_THAT(robot.get_config().x, Catch::Matchers::WithinAbs(1.0, 1e-5));
    REQUIRE_THAT(robot.get_config().y, Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(robot.get_config().w, Catch::Matchers::WithinAbs(0.0, 1e-5));
    turtlelib::DiffDrive robot2 = {1.0, 1.0, {0.0, 0.0}, {0.0, 0.0, 0.0}};
    turtlelib::Wheels updated_wheels = robot2.calculate_IK(new_config);
    REQUIRE_THAT(updated_wheels.phi_l, Catch::Matchers::WithinAbs(1.0, 1e-5));
    REQUIRE_THAT(updated_wheels.phi_r, Catch::Matchers::WithinAbs(1.0, 1e-5));
    turtlelib::DiffDrive robot3 = {1.0, 1.0, {0.0, 0.0}, {0.0, 1.0, 1.0}};
    robot3.calculate_FK(new_phis);
    REQUIRE_THAT(robot3.get_config().x, Catch::Matchers::WithinAbs(2.0, 1e-5));
    REQUIRE_THAT(robot3.get_config().y, Catch::Matchers::WithinAbs(1.0, 1e-5));
    REQUIRE_THAT(robot3.get_config().w, Catch::Matchers::WithinAbs(0.0, 1e-5));


    }

    TEST_CASE("Pure rotation FK and IK", "[transform]" ){ // Liz, Metzger
    turtlelib::DiffDrive robot = {1.0, 1.0, {0.0, 0.0}, {0.0, 0.0, 0.0}};
    turtlelib::Wheels new_phis = {1.0, -1.0};
    turtlelib::Twist2D new_config = {2.0, {0.0, 0.0}};
    robot.calculate_FK(new_phis);
    REQUIRE_THAT(robot.get_track(), Catch::Matchers::WithinAbs(1.0, 0.00001));
    REQUIRE_THAT(robot.get_rad(), Catch::Matchers::WithinAbs(1.0, 0.00001));
    REQUIRE_THAT(robot.get_wheels().phi_l, Catch::Matchers::WithinAbs(-1.0, 1e-5));
    REQUIRE_THAT(robot.get_wheels().phi_r, Catch::Matchers::WithinAbs(1.0, 1e-5));
    REQUIRE_THAT(robot.get_config().x, Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(robot.get_config().y, Catch::Matchers::WithinAbs(0.0, 1e-5));
    REQUIRE_THAT(robot.get_config().w, Catch::Matchers::WithinAbs(2.0, 1e-5));
    turtlelib::DiffDrive robot2 = {1.0, 1.0, {0.0, 0.0}, {0.0, 0.0, 0.0}};
    turtlelib::Wheels updated_wheels = robot2.calculate_IK(new_config);
    REQUIRE_THAT(updated_wheels.phi_l, Catch::Matchers::WithinAbs(-1.0, 1e-5));
    REQUIRE_THAT(updated_wheels.phi_r, Catch::Matchers::WithinAbs(1.0, 1e-5));
    }

    TEST_CASE("Translation and rotation FK and IK", "[transform]" ){ // Liz, Metzger
    turtlelib::DiffDrive robot = {1.0, 1.0, {0.0, 0.0}, {0.0, 0.0, 0.0}};
    turtlelib::Wheels new_phis = {1.0, 2.0};
    turtlelib::Twist2D new_config = {1.0, {1.0, 0.0}};
    robot.calculate_FK(new_phis);
    REQUIRE_THAT(robot.get_track(), Catch::Matchers::WithinAbs(1.0, 0.00001));
    REQUIRE_THAT(robot.get_rad(), Catch::Matchers::WithinAbs(1.0, 0.00001));
    REQUIRE_THAT(robot.get_wheels().phi_l, Catch::Matchers::WithinAbs(2.0, 1e-5));
    REQUIRE_THAT(robot.get_wheels().phi_r, Catch::Matchers::WithinAbs(1.0, 1e-5));
    REQUIRE_THAT(robot.get_config().x, Catch::Matchers::WithinAbs(1.2622064772, 1e-5));
    REQUIRE_THAT(robot.get_config().y, Catch::Matchers::WithinAbs(-0.6895465412, 1e-5));
    REQUIRE_THAT(robot.get_config().w, Catch::Matchers::WithinAbs(-1, 1e-5));
    turtlelib::DiffDrive robot2 = {1.0, 1.0, {0.0, 0.0}, {0.0, 0.0, 0.0}};
    turtlelib::Wheels updated_wheels = robot2.calculate_IK(new_config);
    REQUIRE_THAT(updated_wheels.phi_l, Catch::Matchers::WithinAbs(0.5, 1e-5));
    REQUIRE_THAT(updated_wheels.phi_r, Catch::Matchers::WithinAbs(1.5, 1e-5));
    turtlelib::DiffDrive robot3 = {4.0, 1.0, {0.0, 0.0}, {0.0, 0.0, 0.0}};
    turtlelib::Wheels new_phis2 = {2.0, 1.0};
    robot3.calculate_FK(new_phis2);
    REQUIRE_THAT(robot3.get_config().x, Catch::Matchers::WithinAbs(1.4844237555, 1e-5));
    REQUIRE_THAT(robot3.get_config().y, Catch::Matchers::WithinAbs(0.1865254697, 1e-5));
    REQUIRE_THAT(robot3.get_config().w, Catch::Matchers::WithinAbs(0.25, 1e-5));
    }

    TEST_CASE("Test IK for impossible twist", "[transform]" ){ // Liz, Metzger
    turtlelib::Twist2D new_config = {1.0, {1.0, 1.0}};
    turtlelib::DiffDrive robot2 = {1.0, 1.0, {0.0, 0.0}, {0.0, 0.0, 0.0}};
    REQUIRE_THROWS_AS(robot2.calculate_IK(new_config), std::logic_error);
    }

    TEST_CASE("arc of a circle", "[diffdrive]") { //Nick Morales
        DiffDrive robot {0.16, 0.033};
        Wheels wheels {3.5, 5.0};
        SECTION("forward kinematics") {
            //Update configuration using new wheel position
            robot.calculate_FK(wheels);
            //Check that configuration matches expected configuration
            REQUIRE_THAT(robot.get_config().x, 
                Catch::Matchers::WithinAbs(0.138023393683395, 1e-5));
            REQUIRE_THAT(robot.get_config().y, 
                Catch::Matchers::WithinAbs(-0.0215224326983102, 1e-5));
            REQUIRE_THAT(robot.get_config().w, 
                Catch::Matchers::WithinAbs(-0.309375, 1e-5));
            REQUIRE_THAT(robot.get_wheels().phi_r, 
                Catch::Matchers::WithinAbs(3.5, 1e-5));
            REQUIRE_THAT(robot.get_wheels().phi_l, 
                Catch::Matchers::WithinAbs(5.0, 1e-5));
        }
    }

    TEST_CASE("values from my node", "[diffdrive]") { //Liz Metzger
        DiffDrive robot {0.160, 0.066};
        Wheels wheels {0.076687, -0.076687};
        SECTION("forward kinematics") {
            //Update configuration using new wheel position
            robot.calculate_FK(wheels);
            //Check that configuration matches expected configuration
            REQUIRE_THAT(robot.get_config().x, 
                Catch::Matchers::WithinAbs(0.0, 1e-5));
            REQUIRE_THAT(robot.get_config().y, 
                Catch::Matchers::WithinAbs(0.0, 1e-5));
            REQUIRE_THAT(robot.get_config().w, 
                Catch::Matchers::WithinAbs(0.063266775, 1e-5));
            REQUIRE_THAT(robot.get_wheels().phi_r, 
                Catch::Matchers::WithinAbs(0.076687, 1e-5));
            REQUIRE_THAT(robot.get_wheels().phi_l, 
                Catch::Matchers::WithinAbs(-0.076687, 1e-5));
        }
    }
}
