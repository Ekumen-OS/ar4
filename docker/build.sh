#!/bin/bash
set -e

# Prints information about usage.
function show_help() {
  echo $'\nUsage:\t build.sh [OPTIONS] \n
  Options:\n
  \t-i --image_name\t\t Name of the image to be built (default ar4_ws_ubuntu_jammy).\n
  \t-w --workspace_name\t Name of the workspace folder (default is ar4_ws).\n
  Example:\n
  \tbuild.sh --image_name custom_image_name --workspace_name ar4_ws \n'
}

echo "Building the docker image"

SCRIPT_FOLDER_PATH="$(cd "$(dirname "$0")"; pwd)"
AR4_FOLDER_PATH="$(cd "$(dirname "$0")"; cd .. ; pwd)"

# Parse arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -i|--image_name) IMAGE_NAME="${2}"; shift ;;
        -w|--workspace_name) WORKSPACE_NAME="${2}"; shift ;;
        -h|--help) show_help ; exit 1 ;;
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

OS_VERSION=jammy
IMAGE_NAME=${IMAGE_NAME:-ar4_ws_ubuntu_${OS_VERSION}}
DOCKERFILE_PATH=$SCRIPT_FOLDER_PATH/Dockerfile

WORKSPACE_NAME=${WORKSPACE_NAME:-ar4_ws}

USERID=$(id -u)
USER=$(whoami)

sudo docker build -t $IMAGE_NAME \
     --file $DOCKERFILE_PATH \
     --build-arg USERID=$USERID \
     --build-arg USER=$USER \
     --build-arg WORKSPACE_NAME=$WORKSPACE_NAME \
     $AR4_FOLDER_PATH
