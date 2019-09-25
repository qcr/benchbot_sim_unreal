load("@com_nvidia_isaac//engine/build:isaac.bzl", "isaac_http_archive", "isaac_new_http_archive", "isaac_new_local_repository")

def benchbot_simulator_workspace():
  isaac_new_local_repository(
      name = "ros",
      # path = "/opt/ros/melodic",
      path = "/home/ben/tmp/ros_debug",
      build_file = "//ros:BUILD.bazel",
      licenses = ["https://docs.ros.org/diamondback/api/licenses.html"]
      )

  isaac_http_archive(
      name = "com_google_absl",
      urls = ["https://github.com/abseil/abseil-cpp/archive/20190808.tar.gz"],
      strip_prefix = "abseil-cpp-20190808",
      sha256 = "8100085dada279bf3ee00cd064d43b5f55e5d913be0dfe2906f06f8f28d5b37e",
      licenses = ["@com_google_absl//:COPYRIGHT"]
      )

