load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "rotation_manifold",
    srcs = ["rotation_manifold.h"],
    hdrs = ["rotation_manifold.hpp"],
    deps = [
        ":quaternion_utils",
        ":types",
        "@ceres-solver//:ceres",
        "@eigen",
        "@sophus",
    ],
)

cc_library(
    name = "quaternion_utils",
    srcs = ["quaternion_utils.hpp"],
    hdrs = ["quaternion_utils.h"],
    deps = [
        "@eigen",
        "@glog",
        "@sophus",
    ],
)

cc_library(
    name = "types",
    hdrs = ["types.h"],
)

cc_test(
    name = "quaternion_utils_test",
    srcs = ["quaternion_utils_test.cpp"],
    deps = [
        ":quaternion_utils",
        "@googletest//:gtest",
        "@googletest//:gtest_main",
    ],
)
