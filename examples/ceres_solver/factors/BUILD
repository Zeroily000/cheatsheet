load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "between_factor",
    srcs = ["between_factor.cpp"],
    hdrs = ["between_factor.hpp"],
    deps = [
        # "//examples/ceres_solver/common:pose3_manifolds",
        "@ceres-solver//:ceres",
        "@eigen",
        "@sophus",
    ],
)
