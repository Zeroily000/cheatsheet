#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "modules/employer/employer.h"

namespace py = pybind11;

PYBIND11_MODULE(pyemployer, m) {
    py::class_<Employer>(m, "Employer")
        .def(py::init<>())
        .def("add_employee", &Employer::AddEmployee)
        .def("get_employee", py::overload_cast<int>(&Employer::GetEmployee, py::const_))
        .def("get_employee", py::overload_cast<std::string const &>(&Employer::GetEmployee, py::const_));
}
