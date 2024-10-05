load("@rules_cc//cc:defs.bzl", "cc_binary")

cc_binary(
  name = "main",
  srcs = ["main.cpp"],
  deps = [
    "@opencv",
  ],
  data = [
    "//data:lena",
  ],
)

cc_binary(
  name = "viz3d",
  srcs = ["viz3d.cpp"],
  deps = [
    "@opencv",
  ],
)
