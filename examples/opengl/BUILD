load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

cc_library(
    name = "shader",
    srcs = ["shader.cpp"],
    hdrs = ["shader.hpp"],
    deps = [
        "//third_party/glad",
        "@glfw",
    ],
)

cc_binary(
    name = "main",
    srcs = ["main.cpp"],
    data = [
        "//examples/opengl/shaders:default",
    ],
    deps = [
        ":shader",
        "//third_party/glad",
        "@glfw",
    ],
)
