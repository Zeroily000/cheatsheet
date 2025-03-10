load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

# cc_library(
#     name = "foo",
#     srcs = ["foo.cppm"],
# )

# cc_binary(
#     name = "main",
#     srcs = ["main.cpp"],
#     deps = [
#       ":foo"
#     ],
# )

cc_library(
    name = "foo",
    copts = ["-std=c++20"],
    features = ["cpp_modules"],
    module_interfaces = [
      "foo.cppm"
    ],
)
