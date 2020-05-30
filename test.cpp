#include "kalman.h"
#include <Eigen/Dense>
#include <iostream>
using namespace Eigen;
int main(){
    VectorXf x(3);
    x << 1, 2, 3;
    MatrixXf A = MatrixXf::Identity(3, 3);
    MatrixXf cov = MatrixXf::Identity(3, 3);
    MatrixXf C(2, 3);
    C << 2, 0, 1, 1, 1, 0;
    MatrixXf W = 0 * MatrixXf::Random(3, 3);
    MatrixXf V = 0 * MatrixXf::Random(2, 2);
    KalmanFilter filter(A, C, W, V, x, cov);
    VectorXf y(2);
    for (int i = 0; i < 20; ++i){
        filter.predict();
        std::cout << filter.get_state() << std::endl;
        std::cout << filter.get_cov() << std::endl;
        y = C * x ;
        x = A * x ;
        filter.update(y);
        std::cout << filter.get_state() << std::endl;
        std::cout << filter.get_cov() << std::endl;
    }
}