#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "modules/greet/greet.h"

namespace py = pybind11;

PYBIND11_MODULE(my_pylib, m) {
    py::class_<Greet>(m, "Greet")
        .def(py::init<>())
        .def("__call__", [](Greet & greet) {greet();});
}
