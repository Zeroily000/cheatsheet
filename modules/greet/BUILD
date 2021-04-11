load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "greet",
    srcs = ["greet.cxx"],
    hdrs = ["greet.h"],
    deps = [
        "//modules/hello:hello",
        "//modules/world:world"
    ],
    visibility = ["//visibility:public"]
)

# ==>
# bazel-bin/modules/greet/libgreet.a
# bazel-bin/modules/greet/libgreet.so