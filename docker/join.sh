#!/bin/bash

# Join an open ar4 container session

set +e

HELP="Usage: $me [-s|--service <service>]"
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
    *)
        echo "Invalid argument: $1"
        echo $HELP
        exit 1
        ;;
    esac
done


cd $(dirname $0)

export USERNAME=$(whoami)
export USERID=$(id -u)

docker compose exec -it $SERVICE bash
