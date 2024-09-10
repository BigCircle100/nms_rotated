
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/operators.h>
#include <pybind11/numpy.h>

#include "head.h"

namespace py = pybind11;

PYBIND11_MODULE(algo, m){
  m.def("nms_rotated", &nms_rotated, py::arg("boxes"), py::arg("scores"), py::arg("threshold"));

}