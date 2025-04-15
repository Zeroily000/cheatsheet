load("@rules_cc//cc:defs.bzl", "cc_binary")

cc_binary(
    name = "cpu_rendering",
    srcs = ["cpu_rendering.cpp"],
    data = [
        "//data:lena",
    ],
    deps = [
        "@opencv",
    ],
)

cc_binary(
    name = "gpu_rendering",
    srcs = ["gpu_rendering.cpp"],
    deps = [
        "@opencv",
    ],
)
