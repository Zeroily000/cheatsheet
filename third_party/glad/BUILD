load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "glad",
    srcs = [
        "src/glad.c",
    ],
    hdrs = [
        "include/KHR/khrplatform.h",
        "include/glad/glad.h",
    ],
    includes = [
        "include",
    ],
)
