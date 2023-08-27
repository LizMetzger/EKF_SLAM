#include"turtlelib/make_circles.hpp"
#include<catch2/catch_test_macros.hpp>
#include<catch2/matchers/catch_matchers_floating_point.hpp>
#include<sstream>
#include<iostream>
#include<vector>

namespace turtlelib{
    TEST_CASE("Find a centroid", "[make_circles]" ){ // Liz, Metzger
        std::vector<Point> cluster = {{1,7},{2,6},{5,8},{7,7},{9,5},{3,7}};
        Point centroid = find_centroid(cluster);
        REQUIRE_THAT(centroid.x, 
                    Catch::Matchers::WithinAbs(4.5, 1e-5));
        REQUIRE_THAT(centroid.y, 
                    Catch::Matchers::WithinAbs(6.66666, 1e-5));
        std::vector<Point> cluster2 = {{-1,0.0},{-0.3,-0.06},{0.3,0.1},{1,0}};
        Point centroid2 = find_centroid(cluster2);
        REQUIRE_THAT(centroid2.x, 
                    Catch::Matchers::WithinAbs(0.0, 1e-5));
        REQUIRE_THAT(centroid2.y, 
                    Catch::Matchers::WithinAbs(0.01, 1e-5));
    }

    TEST_CASE("find z_bar", "[make_circles]" ){ // Liz, Metzger
        std::vector<Point> cluster = {{1,7},{2,6},{5,8},{7,7},{9,5},{3,7}};
        double z_bar = find_z_bar(cluster);
        REQUIRE_THAT(z_bar, 
                    Catch::Matchers::WithinAbs(73.5, 1e-5));
        std::vector<Point> cluster2 = {{-1,0.0},{-0.3,-0.06},{0.3,0.1},{1,0}};
        double z_bar2 = find_z_bar(cluster2);
        REQUIRE_THAT(z_bar2, 
                    Catch::Matchers::WithinAbs(0.5484, 1e-5));
    }

    TEST_CASE("test z_mat size", "[make_circles]" ){ // Liz, Metzger
        std::vector<Point> cluster = {{1,7},{2,6},{5,8},{7,7},{9,5},{3,7}};
        arma::mat Z= find_Z_mat(cluster);
        REQUIRE_THAT(Z.n_rows, 
                    Catch::Matchers::WithinAbs(6, 1e-5));
        REQUIRE_THAT(Z.n_cols, 
                    Catch::Matchers::WithinAbs(4, 1e-5));
    }

    TEST_CASE("Test circle", "[make_circles]" ){ // Liz, Metzger
        std::vector<Point> cluster = {{1,7},{2,6},{5,8},{7,7},{9,5},{3,7}};
        Circle circle = find_circles(cluster);
        REQUIRE_THAT(circle.center.x, 
                    Catch::Matchers::WithinAbs(4.615482, 10e-5));
        REQUIRE_THAT(circle.center.y, 
                    Catch::Matchers::WithinAbs(2.807354, 10e-5));
        REQUIRE_THAT(circle.radius, 
                    Catch::Matchers::WithinAbs(4.8275, 10e-5));
        std::vector<Point> cluster2 = {{-1,0.0},{-0.3,-0.06},{0.3,0.1},{1,0}};
        Circle circle2 = find_circles(cluster2);
        REQUIRE_THAT(circle2.center.x, 
                    Catch::Matchers::WithinAbs(0.4908357, 10e-5));
        REQUIRE_THAT(circle2.center.y, 
                    Catch::Matchers::WithinAbs(-22.15212, 10e-5));
        REQUIRE_THAT(circle2.radius, 
                    Catch::Matchers::WithinAbs(22.17979, 10e-5));
    }
}