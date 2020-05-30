#include "kalman.h"
KalmanFilter::KalmanFilter(MatrixXf& A, MatrixXf& C, MatrixXf& W, MatrixXf& V,
                           VectorXf& x_0, MatrixXf& cov_0)
    : A(A),
      C(C),
      W(W),
      V(V),
      state(x_0),
      cov(cov_0),
      state_dim(A.rows()),
      obs_dim(C.rows()) {}
//
void KalmanFilter::predict() {
  state = A * state;
  cov = A * cov * A.transpose() + W;
}
void KalmanFilter::update(VectorXf& y) {
  VectorXf gain(obs_dim);
  gain = C * state - y;
  // Kalman gain matrix cov C^T (C Cov C^T + V)^{-1}
  MatrixXf K = cov * C.transpose() * (C * cov * C.transpose() + V).inverse();
  state += K * gain;
  cov = K * C * cov;
}
VectorXf KalmanFilter::get_state() { return state; }
MatrixXf KalmanFilter::get_cov() { return cov; }