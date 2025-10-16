load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

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
        ":sized_parameter_block",
        "@ceres-solver//:ceres",
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
