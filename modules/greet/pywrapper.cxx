#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

#include "modules/greet/greet.h"

namespace py = pybind11;

PYBIND11_MODULE(pygreet, m) {
    py::class_<Greet>(m, "Greet")
        .def(py::init<>())
        .def("__call__", [](Greet & greet) {greet();});
}
