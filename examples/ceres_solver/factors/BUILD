load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "between_factor",
    srcs = ["between_factor.hpp"],
    hdrs = ["between_factor.h"],
    deps = [
        "//examples/ceres_solver/common:quaternion_utils",
        "//examples/ceres_solver/common:types",
        "@ceres-solver//:ceres",
        "@eigen",
        "@sophus",
    ],
)

cc_library(
    name = "prior_factor",
    srcs = ["prior_factor.hpp"],
    hdrs = ["prior_factor.h"],
    deps = [
        "//examples/ceres_solver/common:quaternion_utils",
        "//examples/ceres_solver/common:types",
        "@ceres-solver//:ceres",
        "@eigen",
        "@sophus",
    ],
)

cc_test(
    name = "between_factor_test",
    srcs = ["between_factor_test.cpp"],
    deps = [
        ":between_factor",
        "//examples/ceres_solver/common:rotation_manifold",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
        "@sophus",
    ],
)

cc_test(
    name = "prior_factor_test",
    srcs = ["prior_factor_test.cpp"],
    deps = [
        ":prior_factor",
        "@sophus",
        "//examples/ceres_solver/common:rotation_manifold",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)
