load("@protobuf//bazel:proto_library.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_binary")

proto_library(
    name = "foo_proto",
    srcs = ["foo.proto"],
    deps = [
        "@protobuf//:timestamp_proto",
    ],
)

cc_proto_library(
    name = "foo_proto_cc",
    deps = [
        ":foo_proto",
    ],
)

cc_binary(
    name = "main",
    srcs = ["main.cpp"],
    deps = [
        ":foo_proto_cc",
        "@protobuf",
        # "@protobuf//src/google/protobuf/util:time_util",
    ],
)
