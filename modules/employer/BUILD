load("@pybind11_bazel//:build_defs.bzl", "pybind_extension")
load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "employer",
    srcs = ["employer.cpp"],
    hdrs = ["employer.hpp"],
    deps = [
        "//modules/common:id_generator",
        "//modules/employee",
        "@abseil-cpp//absl/status:statusor",
        "@boost.multi_index",
    ],
)

pybind_extension(
    name = "pyemployer",
    srcs = ["pyregister.cpp"],
    deps = [
        "//modules/employer",
        "@pybind11_abseil//pybind11_abseil:status_casters",
    ],
)
