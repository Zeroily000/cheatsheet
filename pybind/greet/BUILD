load("@pybind11_bazel//:build_defs.bzl", "pybind_extension")

pybind_extension(
    name = "my_pylib",
    srcs = ["wrapper.cxx"],
    deps = ["//modules/greet:greet"]
)

py_test(
    name = "test",
    srcs = ["test.py"],
    data = [":my_pylib.so"],
    python_version = "PY3",
    srcs_version = "PY3",
)