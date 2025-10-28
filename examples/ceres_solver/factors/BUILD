load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "between_factor",
    srcs = ["between_factor.h"],
    hdrs = ["between_factor.hpp"],
    deps = [
        ":utils",
        "//examples/ceres_solver/common:rotation_manifold",
        "@ceres-solver//:ceres",
        "@eigen",
        "@sophus",
    ],
)

cc_library(
    name = "utils",
    srcs = ["utils.h"],
    hdrs = ["utils.hpp"],
    deps = [
        # "//examples/ceres_solver/common:rotation_manifold",
        # "@ceres-solver//:ceres",
        "@eigen",
        "@sophus",
    ],
)

cc_test(
    name = "between_factor_test",
    srcs = ["between_factor_test.cpp"],
    deps = [
        ":between_factor",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)
