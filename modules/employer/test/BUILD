py_test(
    name = "test",
    srcs = ["test.py"],
    data = [
        "//modules/employee:pyemployee.so",
        "//modules/employer:pyemployer.so"
    ],
    imports = [
        "//modules/employee:pyemployee.so",
        "//modules/employer:pyemployer.so"
    ],
    python_version = "PY3",
    srcs_version = "PY3",
)