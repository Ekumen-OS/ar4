#!/bin/bash

# Run and/or build a service in the ar4 container
# Defaults to ar4_gazebo service

set +e

HELP="Usage: $me [-b|--build] [-s|--service <service>]"
SERVICE="ar4_gazebo"

while [[ "$1" != "" ]]; do
    case "$1" in
    -h | --help)
        echo $HELP
        exit 0
        ;;
    -s | --service)
        SERVICE=$2
        shift 2
        ;;
    -b | --build)
        BUILD=true
        shift
        ;;
    *)
        echo "Invalid argument: $1"
        echo $HELP
        exit 1
        ;;
    esac
done


cd $(dirname $0)

export USERID=$(id -u)
export GROUPID=$(id -g)

docker compose run --rm ${BUILD:+--build} $SERVICE
