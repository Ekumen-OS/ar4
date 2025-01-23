#!/usr/bin/env bash

apt-get update && apt-get install --no-install-recommends -y $(cat requirements_ros.txt)
