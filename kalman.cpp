#include "kalman.h"
KalmanFilter::KalmanFilter(){state_dim = 0; obs_dim=0;}
KalmanFilter::KalmanFilter(MatrixXf A, MatrixXf C, MatrixXf W, MatrixXf V,
                           VectorXf x_0, MatrixXf cov_0)
    : state_dim(A.rows()),
      obs_dim(C.rows()),
      A(A),
      C(C),
      W(W),
      V(V),
      state(x_0),
      cov(cov_0)
      {}
//
void KalmanFilter::set(MatrixXf A_, MatrixXf C_, MatrixXf W_, MatrixXf V_,
                    VectorXf x_0_, MatrixXf cov_0_){
      A = A_;
      C = C_;
      W = W_;
      V = V_;
      state = x_0_;
      cov = cov_0_;
      state_dim = A_.rows();
      obs_dim = C_.rows();
}
void KalmanFilter::predict() {
  state = A * state;
  cov = A * cov * A.transpose() + W;
}
void KalmanFilter::update(VectorXf y) {
  VectorXf gain(obs_dim);
  gain = y - C * state;
  // Kalman gain matrix cov C^T (C Cov C^T + V)^{-1}
  MatrixXf K = cov * C.transpose() * (C * cov * C.transpose() + V).inverse();
  state += K * gain;
  cov -= K * C * cov;
}
VectorXf KalmanFilter::get_state() { return state; }
MatrixXf KalmanFilter::get_cov() { return cov; }