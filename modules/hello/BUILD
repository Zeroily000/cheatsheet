load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "hello",
    srcs = ["hello.cpp"],
    hdrs = ["hello.hpp"],
)

# ==>
# bazel-bin/modules/hello/libhello.a
# bazel-bin/modules/hello/libhello.so