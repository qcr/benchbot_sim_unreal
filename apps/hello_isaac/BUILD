load("@com_nvidia_isaac//engine/build:isaac.bzl", "isaac_app", "isaac_cc_module")

isaac_cc_module(
    name = "ping_isaac",
    srcs = ["IsaacPinger.cc"],
    hdrs = ["IsaacPinger.hh"],
    visibility = ["//visibility:public"]
    )

isaac_app(
    name = "hello_isaac",
    modules = ["//apps/hello_isaac:ping_isaac"]
    )


# cc_binary(
#     name = "hello_isaac",
#     srcs = ["hello_isaac.cc"],
#     deps = ["@com_google_absl//absl/strings"]
#     )

