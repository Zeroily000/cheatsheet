load("@rules_cc//cc:defs.bzl", "cc_library")
load("@pybind11_bazel//:build_defs.bzl", "pybind_extension")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "employee",
    srcs = ["employee.cxx"],
    hdrs = ["employee.h"],
)

pybind_extension(
    name = "pyemployee",
    srcs = ["pywrapper.cpp"],
    deps = [
        "//modules/employee",
    ]
)