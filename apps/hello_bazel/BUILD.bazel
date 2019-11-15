cc_binary(
    name = "hello_bazel",
    srcs = ["hello_bazel.cc"],
    deps = [
        ":hello_library",
        ],
    )

cc_library(
    name = "hello_library",
    srcs = ["hello_library.cc"],
    hdrs = ["hello_library.h"],
    visibility = ["//visibility:public"],
    )
