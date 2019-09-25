#!/bin/bash

ROOT_PATH="$( cd "$(dirname "$0")"; cd .. ; pwd -P )"
BASE_TAG=1.12.0-kinetic

# Autoware image with CUDA
echo "BUILDING AUTOWARE CUDA IMAGE"

docker build \
       -f $ROOT_PATH/docker/Dockerfile.base \
       -t ppavp-sd-test:base-cuda-$BASE_TAG \
       --build-arg FROM_ARG=${BASE_TAG}-cuda \
       $ROOT_PATH
