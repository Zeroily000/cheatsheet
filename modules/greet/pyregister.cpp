#include "pybind11/pybind11.h"

#include "modules/greet/greet.hpp"

namespace py = pybind11;

PYBIND11_MODULE(pygreet, m) {
    py::class_<Greet>(m, "Greet")
        .def(py::init<>())
        .def("__call__", [](Greet & greet) {greet();});
}
