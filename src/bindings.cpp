#include <pybind11/pybind11.h>

#include "../include/msckf.hpp"
#include "../include/types.hpp"

namespace py = pybind11;

PYBIND11_MODULE(msckf, m)
{
    py::class_<msckf_mono::MSCKF<double>>(m, "MSCKF")
        .def(py::init<>());

    py::class_<msckf_mono::Camera<double>>(m, "Camera")
        .def(py::init<>());
}