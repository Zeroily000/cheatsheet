load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "rotation_manifold",
    srcs = ["rotation_manifold.h"],
    hdrs = ["rotation_manifold.hpp"],
    deps = [
        "@ceres-solver//:ceres",
        "@eigen",
        "@sophus",
    ],
)
