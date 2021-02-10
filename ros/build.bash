#!/usr/bin/env bash

# Settings
ROS_DISTRO="melodic"
PACKAGE_FILE=$(realpath "$1")
ROS_BUILD="ros_build"
ROS_BUILT="ros_built"

ROS_OUT_LIST="$(realpath "$2")"
ROS_OUT_DIR=$(dirname "$ROS_OUT_LIST")
ROS_CP_DIR="$(realpath "$3")"

echo "Loading packages from: $PACKAGE_FILE"
echo "Building in: $ROS_OUT_DIR"
echo "Saving files list: $ROS_OUT_LIST"
echo "Coping files to: $ROS_CP_DIR"

# Pull the list of packages
packages=$(cat "$PACKAGE_FILE" | tr "\n" " ")
echo "Manually pulling headers & building libraries from the following ROS packages:"
echo "$packages"

# Ensure we have pip dependencies
pip install trollius catkin_tools

# Install into our temporary dumping ground...
set -e
pushd "$ROS_OUT_DIR"
rm -rf "$ROS_BUILD"
mkdir -v "$ROS_BUILD"
pushd "$ROS_BUILD"

rosinstall_generator \
    --rosdistro "$ROS_DISTRO" \
    --deps \
    --flat \
    $packages > ws.rosinstall
wstool init -j8 src ws.rosinstall

git clone --branch develop https://github.com/qcr/benchbot_msgs.git src/benchbot_msgs

catkin config \
    --install \
    --source-space src \
    --build-space build \
    --devel-space devel \
    --log-space log \
    --install-space install \
    --isolate-devel \
    --no-extend

catkin build

# Put everything into our completed build directory
popd
rm -rf "$ROS_BUILT"
mkdir -v  "$ROS_BUILT"
pushd "$ROS_BUILT"

cp -r ../"$ROS_BUILD"/install/lib .
cp -r ../"$ROS_BUILD"/install/include .

# Dump the built package list
cp "$PACKAGE_FILE" "$ROS_OUT_LIST"

# Clean up & copy files to output location
# TODO would love to remove... but Bazel really doesn't like dynamically
# generated file lists
popd
rm -rf "$ROS_CP_DIR/lib" "$ROS_CP_DIR/include"
cp -r "$ROS_BUILT"/* "$ROS_CP_DIR"
rm -rf "$ROS_BUILD" "$ROS_BUILT"
