load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "pose3_manifolds",
    srcs = ["pose3_manifolds.cpp"],
    hdrs = ["pose3_manifolds.hpp"],
    deps = [
        "@ceres-solver//:ceres",
        "@eigen",
        "@sophus",
    ],
)

cc_library(
    name = "sized_parameter_block",
    srcs = ["sized_parameter_block.h"],
    hdrs = ["sized_parameter_block.hpp"],
    deps = [
        "@glog",
    ],
)

cc_library(
    name = "pose3",
    srcs = ["pose3.h"],
    hdrs = ["pose3.hpp"],
    deps = [
        ":pose3_manifolds",
        ":sized_parameter_block",
        "@eigen",
    ],
)

cc_test(
    name = "pose3_test",
    srcs = ["pose3_test.cpp"],
    deps = [
        ":pose3",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)
