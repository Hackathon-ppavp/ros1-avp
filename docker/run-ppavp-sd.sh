#!/bin/bash

split_opts_cmd() {
    OPTS=()

    while [[ $# -gt 0 ]]
    do
        case $1 in
            -h|--help)
                echo "Usage: $0 [--image|-i IMAGE] [--tag|-t TAG] [ADDITIONAL_DOCKER_OPTIONS]"
                exit
                ;;
            -t|--tag)
                TAG="$2"
                shift
                shift
                ;;
            -i|--image)
                IMAGE="$2"
                shift
                shift
                ;;
            *)
                OPTS+=("$1")
                shift
                ;;
        esac
    done


    DOCKER_OPTS="${OPTS[@]}"
}

until docker ps > /dev/null
do
    echo "Waiting for docker server"
    sleep 1
done

echo "Ensuring devel and build exist..."

ROOT_PATH="$( cd "$(dirname "$0")"; cd .. ; pwd -P )"
mkdir -p ${ROOT_PATH}/ros/devel
mkdir -p ${ROOT_PATH}/ros/build

IMAGE=ppavp-sd-test
TAG=base-cuda-1.12.0-kinetic

XSOCK=/tmp/.X11-unix
XAUTH=/home/$USER/.Xauthority
RUNTIME="--runtime=nvidia"

split_opts_cmd "$@"
C="${CMD[@]}"


echo "Starting docker container ($IMAGE:$TAG)..."

echo "Docker options:"
echo ${DOCKER_OPTS[@]}

docker run $RUNTIME \
              -it --rm \
              --volume=$XSOCK:$XSOCK:rw \
              --volume=$XAUTH:$XAUTH:rw \
              --env="XAUTHORITY=${XAUTH}" \
              --env="DISPLAY=${DISPLAY}" \
			  --privileged --device=/dev/input/js0 \
			  --net host \
              --env="USER_ID=$(id -u)" \
              -v ${ROOT_PATH}/ros:/home/autoware/ppavp \
              ${DOCKER_OPTS[@]} \
              $IMAGE:$TAG
