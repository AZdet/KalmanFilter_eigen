#include <Eigen/Dense>
using namespace Eigen;
class KalmanFilter {
 public:
  KalmanFilter();
  KalmanFilter(MatrixXf A, MatrixXf C, MatrixXf W, MatrixXf V,
                    VectorXf x_0, MatrixXf cov_0);
  void set(MatrixXf A, MatrixXf C, MatrixXf W, MatrixXf V,
                    VectorXf x_0, MatrixXf cov_0);
  void predict();
  void update(VectorXf y);
  VectorXf get_state();
  MatrixXf get_cov();

 private:
  size_t state_dim, obs_dim;
  MatrixXf A, C, W, V;
  VectorXf state;
  MatrixXf cov;
  // const
};
