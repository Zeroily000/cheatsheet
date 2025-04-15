#include "pybind11_abseil/status_casters.h"
#include "pybind11/pybind11.h"

#include "modules/employer/employer.hpp"

namespace py = pybind11;

PYBIND11_MODULE(pyemployer, m) {
  pybind11::google::ImportStatusModule();
  py::class_<Employer>(m, "Employer")
    .def(py::init<>())
    .def("add_employee", &Employer::AddEmployee)
    .def("remove_employee", &Employer::RemoveEmployee)
    .def("get_employee", py::overload_cast<int>(&Employer::GetEmployee, py::const_))
    .def("get_employee", py::overload_cast<std::string const &>(&Employer::GetEmployee, py::const_));
}
