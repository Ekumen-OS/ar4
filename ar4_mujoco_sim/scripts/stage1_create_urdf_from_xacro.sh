#!/bin/bash

set +e

# this script must only be executed when WITHIN_DEV is set to 1
if [ "$WITHIN_DEV" != "1" ]; then
    echo "This stage of the MJCF creation process must be executed within the development environment."
    exit 1
fi

# assing to the BASEDIR variable the directory where the script is located
BASEDIR=$(dirname $0)

pushd /home/developer/ws

# Build the repository from scratch
# rm -rf build/* install/*
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# Create a temporary directory to store the urdf
mkdir -p /tmp/urdf
rm -rf /tmp/urdf/*

# Create the urdf and copy the meshes with it
xacro src/project/ar4_mujoco_sim/xacro/mj_ar4.urdf.xacro > /tmp/urdf/mj_ar4.urdf
cp -r src/project/ar4_description/meshes /tmp/urdf

# Make paths in the urdf relative
REMOVED_PREFIX='file:\/\/\/home\/developer\/ws\/install\/ar4_description\/share\/ar4_description\/'
sed -i "s/$REMOVED_PREFIX//" /tmp/urdf/mj_ar4.urdf

popd

# update the urdf in this package with the new one
mkdir -p $BASEDIR/../urdf
rm -rf $BASEDIR/../urdf/*
cp -r /tmp/urdf/* $BASEDIR/../urdf
