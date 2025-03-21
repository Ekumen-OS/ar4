#!/bin/bash

set +e

# this script must only be executed when WITHIN_DEV is unset
if [ "$WITHIN_DEV" != "" ]; then
    echo "This stage of the MJCF creation process must be executed outside the development environment."
    exit 1
fi

# assing to the BASEDIR variable the directory where the script is located
BASEDIR=$(dirname $0)

# get the directory of the package
PKG_DIR=$(realpath $BASEDIR/..)

# input and output directories
META_DIR=$(realpath $PKG_DIR/meta)
URDF_DIR=$(realpath $PKG_DIR/urdf)
MJCF_DIR=$(realpath $PKG_DIR/mjcf/robot)

IN_CONTAINER_WORKDIR=/home/developer

mkdir -p $MJCF_DIR

true \
 && docker build \
   -t urjc-builder \
   -f docker/Dockerfile \
   --build-arg USERID=$(id -u) \
   --build-arg GROUPID=$(id -g) \
   . \
 && docker run -it \
    -v $META_DIR:$IN_CONTAINER_WORKDIR/meta \
    -v $URDF_DIR:$IN_CONTAINER_WORKDIR/urdf \
    -v $MJCF_DIR:$IN_CONTAINER_WORKDIR/mjcf \
    urjc-builder:latest \
    "$@"
