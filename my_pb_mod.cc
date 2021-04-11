#include <pybind11/pybind11.h>

namespace {

int add(int x, int y) {
    return x + y;
}

} // namespace

PYBIND11_MODULE(my_pylib, m) {
    m.def("add", &add, "adds two numbers");
}