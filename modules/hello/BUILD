load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "hello",
    srcs = ["hello.cxx"],
    hdrs = ["hello.h"],
    visibility = ["//visibility:public"]
)

# ==>
# bazel-bin/modules/hello/libhello.a
# bazel-bin/modules/hello/libhello.so