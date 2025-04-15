#include "pybind11/pybind11.h"

#include "modules/employee/employee.hpp"

namespace py = pybind11;

PYBIND11_MODULE(pyemployee, m) {
    py::class_<Employee>(m, "Employee")
        .def_readwrite("id", &Employee::id)
        .def_readwrite("name", &Employee::name);
}
