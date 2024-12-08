#!/bin/bash

# Project: ENPM662-Project1-Group1
# Description: Build a docker image using the Dockerfile

PROJECT=jammy
USERNAME=$(id -un)
USER_ID=$(id -u)
GROUP_ID=$(id -g)
HOST_DISPLAY=`echo ${DISPLAY}`
BA="--build-arg"

if [[ "${USERNAME}" == "root" ]]; then 
    echo -e "This script is not intended to be run as root or with sudo.\n"
    exit 1
fi

# these paramaters allow the Dockerfile to inject your user account into the container.
# which is helpful for mounting host directories to the container and having 
# a non-root user account to log in with.  
PARAMS="${BA} USERNAME=${USERNAME}"
PARAMS="${PARAMS} ${BA} USER_ID=${USER_ID}"
PARAMS="${PARAMS} ${BA} GROUP_ID=${GROUP_ID}"
PARAMS="${PARAMS} ${BA} HOST_DISPLAY=${HOST_DISPLAY}"

# if building image on WSL, make sure WSL graphics work with the container.
if [[ -e /etc/wsl.conf ]]; then
    PARAMS="${PARAMS} ${BA} WSLG=/mnt/wslg/runtime-dir/"
fi

# if "rebuild" paramater used with this script, fully rebuild image.  
if [[ "$1" == "rebuild" ]]; then
    PARAMS="${PARAMS} --no-cache"
fi

#build the image and assume sudo if not using WSL
if [[ -e /etc/wsl.conf ]]; then
    docker build ${PARAMS} -t ${PROJECT} .
else
    sudo docker build ${PARAMS} -t ${PROJECT} .
fi
