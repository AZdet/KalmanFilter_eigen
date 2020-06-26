// <%cppimport
// cfg['compiler_args'] = ['-std=c++11', '-I', '/home/shzhou2/project/eigen-eigen-323c052e1731']
// setup_pybind11(cfg)
// %>
#include "kalman.h"
#include <Eigen/Dense>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>

namespace py = pybind11;
using namespace Eigen;

PYBIND11_MODULE(wrap, m){
    py::class_<KalmanFilter>(m, "KalmanFilter")
    .def(py::init<>())
    .def("set", &KalmanFilter::set)
    .def("predict", &KalmanFilter::predict)
    .def("update", &KalmanFilter::update)
    .def("get_state", &KalmanFilter::get_state)
    .def("get_cov", &KalmanFilter::get_cov);
}
