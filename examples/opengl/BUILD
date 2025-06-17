load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

filegroup(
    name = "shaders",
    srcs = [
        "shader.fs",
        "shader.vs",
    ],
)

cc_library(
    name = "shader",
    srcs = ["shader.cpp"],
    hdrs = ["shader.hpp"],
    data = [
        ":shaders",
    ],
    deps = [
        "//third_party/glad",
        "@glfw",
    ],
)

cc_binary(
    name = "main",
    srcs = ["main.cpp"],
    deps = [
        ":shader",
        "//third_party/glad",
        "@glfw",
    ],
)
