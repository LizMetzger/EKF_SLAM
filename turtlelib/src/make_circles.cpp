#include "turtlelib/make_circles.hpp"

namespace turtlelib{
    double get_dist(double x1, double y1, double x2, double y2){
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
    }

    double check_line(double new_point, double robot_point, double max){
    return (new_point - robot_point) / (max - robot_point);
    }

    Point find_centroid(std::vector<Point> cluster){
        int n = size(cluster);
        double x_sum = 0.0, y_sum = 0.0;
        for (int i = 0; i < n; i++){
            x_sum += cluster.at(i).x;
            y_sum += cluster.at(i).y;
        }
        return {x_sum/n, y_sum/n};
    }

    double find_z_bar(std::vector<Point> shifted_cluster){
        int n = size(shifted_cluster);
        double z_sum = 0.0;
        for (int i = 0; i < n; i++){
            z_sum += shifted_cluster.at(i).x*shifted_cluster.at(i).x + shifted_cluster.at(i).y*shifted_cluster.at(i).y;
        }
        return z_sum/n;
    }

    arma::mat find_Z_mat(std::vector<Point> shifted_cluster){
        int n = size(shifted_cluster);
        arma::mat Z_mat = arma::mat(n, 4, arma::fill::ones);
        for (int i = 0; i < n; i++){
            Z_mat(i, 0) = shifted_cluster.at(i).x*shifted_cluster.at(i).x + shifted_cluster.at(i).y*shifted_cluster.at(i).y;
            Z_mat(i, 1) = shifted_cluster.at(i).x;
            Z_mat(i, 2) = shifted_cluster.at(i).y;
        }
        return Z_mat;
    }

    Circle calc_circle(arma::vec A){
        double a = -A(1)/(2*A(0));
        double b = -A(2)/(2*A(0));
        double r = sqrt((A(1)*A(1) + (A(2)*A(2)) - 4*A(0)*A(3))/(4*A(0)*A(0)));
        return {{a,b},r};
    }

    Circle find_circles(std::vector<Point> cluster){
        // find the centroid
        Point centroid = find_centroid(cluster);
        // std::cout << "centroid:" << centroid.x << " " << centroid.y << "\n";
        // create a shifted vector
        std::vector<Point> shifted_cluster;
        int n = cluster.size();
        for (int i = 0; i < n; i++){
            shifted_cluster.push_back({cluster.at(i).x - centroid.x, cluster.at(i).y - centroid.y});
        }

        // find z_bar
        double z_bar = find_z_bar(shifted_cluster);
        // std::cout << "zbar: " << z_bar << "\n";

        // find Z_mat (data matrix)
        arma::mat Z_mat = find_Z_mat(shifted_cluster);

        // find M_mat (moment matrix)
        arma::mat M_mat = (1/n)*Z_mat.t()*Z_mat;

        // find H_inv (constraint matrix)
        arma::mat H_inv = arma::mat(4, 4, arma::fill::zeros);
        H_inv(3,0) = 0.5;
        H_inv(1,1) = 1;
        H_inv(2,2) = 1;
        H_inv(0,3) = 0.5;
        H_inv(3,3) = -2*z_bar;
        // std::cout << "H_inv" << H_inv << "\n\n\n";

        // singular value decomposition
        arma::mat U;
        arma::vec Sigma;
        arma::mat V;

        // std::cout << "Z: " << Z_mat << "\n\n\n";
        arma::svd(U,Sigma,V,Z_mat);
        // V = V.t();
        // Make A_vec
        arma::vec A;
        // if s_4 is less than 10^-12 then A is the 4th column of V
        if (Sigma(3) < 10e-12){
            A = V.col(3);
        }
        else{
            // get Sigma into a matrix
            arma::mat Sigma_mat = arma::diagmat(Sigma);
            // std::cout << "Sigma matrix:" << Sigma_mat << "\n\n\n";
            // calculate Y
            // std::cout << "V:" << V << "\n\n\n";
            // std::cout << "V.t:" << V.t() << "\n\n\n";
            arma::mat Y = V*Sigma_mat*V.t();
            // std::cout << "Y: " << Y << "\n\n\n\n";
            // calculate Q
            // std::cout << "H_inv" << H_inv << "\n\n\n";
            arma::mat Q = Y*H_inv*Y;
            // std::cout << "Q: " << Q << "\n\n\n";
            // find the eigenvalues and eigenvectors
            arma::cx_vec eigval;
            arma::cx_mat eigvec;
            arma::eig_gen(eigval, eigvec, Q);
            // std::cout << "eigval: " << eigval << "\n\n\n\n";
            // std::cout << "eigvec: " << eigvec << "\n\n\n\n";
            // the smallest positive eigenvalue corresponds to our desired eigen value
            int smallest_ind = 0;
            double smallest_val = 999999;
            for (int j = 0; j < 4; j ++){
                if (eigval.at(j).real() > 0.0 && eigval.at(j).real() < smallest_val){
                    smallest_ind = j;
                    smallest_val = eigval.at(j).real();
                }
            }
            // std::cout << "min eigval: " << smallest_val << "\n";
            // std::cout << "min ind: " << smallest_ind << "\n";
            arma::vec A_star = arma::vec(4);
            for (int k = 0; k < 4; k ++){
                A_star(k) = eigvec(k, smallest_ind).real();
            }
            // std::cout << "A_star: " << A_star << "\n";
            A = Y.i()*A_star;
            // std::cout << "A: " << A << "\n\n\n";
        }

        // solve for a circle 
        Circle circle = calc_circle(A);
        circle.center.x += centroid.x;
        circle.center.y += centroid.y;
        return circle;
    }
}
