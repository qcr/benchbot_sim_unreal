# Modeled from: https://github.com/nvidia-isaac/velodyne_lidar/blob/master/WORKSPACE
# TODO pull paths out into environment variables
workspace(name = "benchbot_simulator")

local_repository(name = "com_nvidia_isaac", path = "/home/ben/opt/isaac_2019-2")

load("@com_nvidia_isaac//third_party:engine.bzl", "isaac_engine_workspace")
load("@com_nvidia_isaac//third_party:packages.bzl", "isaac_packages_workspace")
isaac_engine_workspace()
isaac_packages_workspace()

load("@com_nvidia_isaac//engine/build/toolchain:toolchain.bzl", "toolchain_configure")
toolchain_configure(name = "toolchain")

load(":.repositories.bzl", "benchbot_simulator_workspace")
benchbot_simulator_workspace()
