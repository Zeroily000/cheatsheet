load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "opencv",
    srcs = glob([
        "lib/x86_64-linux-gnu/libopencv_*.so*"
    ]),
    hdrs = glob([
        "include/opencv4/opencv2/**/*.h",
        "include/opencv4/opencv2/**/*.hpp",
    ]),
    includes = [
        "include/opencv4"
    ],
)
