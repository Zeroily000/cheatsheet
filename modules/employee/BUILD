load("@rules_cc//cc:defs.bzl", "cc_library")
load("@pybind11_bazel//:build_defs.bzl", "pybind_extension")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "employee",
    srcs = ["employee.cpp"],
    hdrs = ["employee.hpp"],
)

pybind_extension(
    name = "pyemployee",
    srcs = ["pyregister.cpp"],
    deps = [
        "//modules/employee",
    ]
)