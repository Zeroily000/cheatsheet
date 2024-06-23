load("@rules_cc//cc:defs.bzl", "cc_library")
load("@pybind11_bazel//:build_defs.bzl", "pybind_extension")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "greet",
    srcs = ["greet.cxx"],
    hdrs = ["greet.h"],
    deps = [
        "//modules/hello:hello",
        "//modules/world:world"
    ]
)

pybind_extension(
    name = "pygreet",
    srcs = ["pywrapper.cxx"],
    deps = [
        "//modules/greet:greet"
    ]
)
