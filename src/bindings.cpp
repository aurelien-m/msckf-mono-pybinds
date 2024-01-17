#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "../include/msckf.hpp"
#include "../include/types.hpp"

namespace py = pybind11;

void set_cam_quat(msckf_mono::Camera<double> &cam, Eigen::Matrix<double, 4, 1> q)
{
    cam.q_CI = msckf_mono::Quaternion<double>(q(0), q(1), q(2), q(3));
}

Eigen::Matrix<double, 4, 1> get_cam_quat(msckf_mono::Camera<double> &cam)
{
    return cam.q_CI.coeffs();
}

void set_cam_point(msckf_mono::Camera<double> &cam, Eigen::Matrix<double, 3, 1> p)
{
    cam.p_C_I = msckf_mono::Point<double>(p(0), p(1), p(2));
}

Eigen::Matrix<double, 3, 1> get_cam_point(msckf_mono::Camera<double> &cam)
{
    return cam.p_C_I;
}

void set_imu_state_p(msckf_mono::imuState<double> &imu, Eigen::Matrix<double, 3, 1> p)
{
    imu.p_I_G = msckf_mono::Point<double>(p(0), p(1), p(2));
}

Eigen::Matrix<double, 3, 1> get_imu_state_p(msckf_mono::imuState<double> &imu)
{
    return imu.p_I_G;
}

void set_imu_state_p_null(msckf_mono::imuState<double> &imu, Eigen::Matrix<double, 3, 1> p)
{
    imu.p_I_G_null = msckf_mono::Point<double>(p(0), p(1), p(2));
}

Eigen::Matrix<double, 3, 1> get_imu_state_p_null(msckf_mono::imuState<double> &imu)
{
    return imu.p_I_G_null;
}

void set_imu_state_v(msckf_mono::imuState<double> &imu, Eigen::Matrix<double, 3, 1> v)
{
    imu.v_I_G = msckf_mono::Point<double>(v(0), v(1), v(2));
}

Eigen::Matrix<double, 3, 1> get_imu_state_v(msckf_mono::imuState<double> &imu)
{
    return imu.v_I_G;
}

void set_imu_state_b_g(msckf_mono::imuState<double> &imu, Eigen::Matrix<double, 3, 1> b_g)
{
    imu.b_g = msckf_mono::Point<double>(b_g(0), b_g(1), b_g(2));
}

Eigen::Matrix<double, 3, 1> get_imu_state_b_g(msckf_mono::imuState<double> &imu)
{
    return imu.b_g;
}

void set_imu_state_b_a(msckf_mono::imuState<double> &imu, Eigen::Matrix<double, 3, 1> b_a)
{
    imu.b_a = msckf_mono::Point<double>(b_a(0), b_a(1), b_a(2));
}

Eigen::Matrix<double, 3, 1> get_imu_state_b_a(msckf_mono::imuState<double> &imu)
{
    return imu.b_a;
}

void set_imu_state_g(msckf_mono::imuState<double> &imu, Eigen::Matrix<double, 3, 1> g)
{
    imu.g = msckf_mono::Point<double>(g(0), g(1), g(2));
}

Eigen::Matrix<double, 3, 1> get_imu_state_g(msckf_mono::imuState<double> &imu)
{
    return imu.g;
}

void set_imu_state_v_null(msckf_mono::imuState<double> &imu, Eigen::Matrix<double, 3, 1> v)
{
    imu.v_I_G_null = msckf_mono::Point<double>(v(0), v(1), v(2));
}

Eigen::Matrix<double, 3, 1> get_imu_state_v_null(msckf_mono::imuState<double> &imu)
{
    return imu.v_I_G_null;
}

void set_imu_state_q(msckf_mono::imuState<double> &imu, Eigen::Matrix<double, 4, 1> q)
{
    imu.q_IG = msckf_mono::Quaternion<double>(q(0), q(1), q(2), q(3));
}

Eigen::Matrix<double, 4, 1> get_imu_state_q(msckf_mono::imuState<double> &imu)
{
    return imu.q_IG.coeffs();
}

void set_imu_state_q_null(msckf_mono::imuState<double> &imu, Eigen::Matrix<double, 4, 1> q)
{
    imu.q_IG_null = msckf_mono::Quaternion<double>(q(0), q(1), q(2), q(3));
}

Eigen::Matrix<double, 4, 1> get_imu_state_q_null(msckf_mono::imuState<double> &imu)
{
    return imu.q_IG_null.coeffs();
}

PYBIND11_MODULE(msckf, m)
{
    py::class_<msckf_mono::MSCKF<double>>(m, "MSCKF")
        .def(py::init<>())
        .def("initialize", &msckf_mono::MSCKF<double>::initialize);

    py::class_<msckf_mono::Camera<double>>(m, "Camera")
        .def(py::init<>())
        .def_readwrite("c_u", &msckf_mono::Camera<double>::c_u)
        .def_readwrite("c_v", &msckf_mono::Camera<double>::c_v)
        .def_readwrite("f_u", &msckf_mono::Camera<double>::f_u)
        .def_readwrite("f_v", &msckf_mono::Camera<double>::f_v)
        .def_property("q_CI", &get_cam_quat, &set_cam_quat)
        .def_property("p_C_I", &get_cam_point, &set_cam_point);

    py::class_<msckf_mono::noiseParams<double>>(m, "NoiseParams")
        .def(py::init<>())
        .def_readwrite("u_var_prime", &msckf_mono::noiseParams<double>::u_var_prime)
        .def_readwrite("v_var_prime", &msckf_mono::noiseParams<double>::v_var_prime)
        .def_readwrite("Q_imu", &msckf_mono::noiseParams<double>::Q_imu)
        .def_readwrite("initial_imu_covar", &msckf_mono::noiseParams<double>::initial_imu_covar);

    py::class_<msckf_mono::MSCKFParams<double>>(m, "MSCKFParams")
        .def(py::init<>())
        .def_readwrite("max_gn_cost_norm", &msckf_mono::MSCKFParams<double>::max_gn_cost_norm)
        .def_readwrite("min_rcond", &msckf_mono::MSCKFParams<double>::min_rcond)
        .def_readwrite("translation_threshold", &msckf_mono::MSCKFParams<double>::translation_threshold)
        .def_readwrite("redundancy_angle_thresh", &msckf_mono::MSCKFParams<double>::redundancy_angle_thresh)
        .def_readwrite("redundancy_distance_thresh", &msckf_mono::MSCKFParams<double>::redundancy_distance_thresh)
        .def_readwrite("min_track_length", &msckf_mono::MSCKFParams<double>::min_track_length)
        .def_readwrite("max_track_length", &msckf_mono::MSCKFParams<double>::max_track_length)
        .def_readwrite("max_cam_states", &msckf_mono::MSCKFParams<double>::max_cam_states);

    py::class_<msckf_mono::imuState<double>>(m, "ImuState")
        .def(py::init<>())
        .def_property("p_I_G", &get_imu_state_p, &set_imu_state_p)
        .def_property("p_I_G_null", &get_imu_state_p_null, &set_imu_state_p_null)
        .def_property("v_I_G", &get_imu_state_v, &set_imu_state_v)
        .def_property("b_g", &get_imu_state_b_g, &set_imu_state_b_g)
        .def_property("b_a", &get_imu_state_b_a, &set_imu_state_b_a)
        .def_property("g", &get_imu_state_g, &set_imu_state_g)
        .def_property("v_I_G_null", &get_imu_state_v_null, &set_imu_state_v_null)
        .def_property("q_IG", &get_imu_state_q, &set_imu_state_q)
        .def_property("q_IG_null", &get_imu_state_q_null, &set_imu_state_q_null);
}
