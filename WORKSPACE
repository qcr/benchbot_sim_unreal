# Modeled from: https://github.com/nvidia-isaac/velodyne_lidar/blob/master/WORKSPACE
# TODO pull paths out into environment variables
workspace(name = "benchbot_simulator")

local_repository(name = "nvidia_isaac", path =
                 "/home/ben/opt/isaac_2019-2/")

load("@nvidia_isaac//third_party:engine.bzl", "isaac_engine_workspace")
load("@nvidia_isaac//third_party:packages.bzl", "isaac_packages_workspace")

isaac_engine_workspace()
isaac_packages_workspace()

load("@nvidia_isaac//engine/build/toolchain:toolchain.bzl", "toolchain_configure")

toolchain_configure(name = "toolchain")

# TODO load dependencies specific to this workspace...
load("@nvidia_isaac//engine/build:isaac.bzl", "isaac_new_local_repository")
isaac_new_local_repository(licenses = ["@com_google_absl//:COPYRIGHT"], name =
                           "benchbot_simulator")
