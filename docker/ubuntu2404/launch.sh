#!/bin/bash

# Project: ENPM662-Project1-Group1
# Description: Launch a Container

USERNAME=$(id -un)
USER_ID=$(id -u)
GROUP_ID=$(id -g)
REPO="enpm662p1"
PROJECT='noble'
X11='/tmp/.X11-unix:/tmp/.X11-unix'
WSLG='/mnt/wslg:/mnt/wslg'
PARAMS="-v ${X11}"

if [[ "${USERNAME}" == "root" ]]; then 
    echo -e "This script is not intended to be run as root or with sudo.\n"
    exit 1
fi

# if WSL environment, fixup graphics
if [[ -e /etc/wsl.conf ]]; then
    PARAMS="${PARAMS} -v ${WSLG}"
fi

# try to find project directory and mount to container
MOUNT_DIR="/home/${USERNAME}/cpp/${REPO}"
if [[ ! -d ${MOUNT_DIR} ]]; then
    MOUNT_DIR="/home/${USERNAME}/${REPO}"
elif [[ ! -d ${MOUNT_DIR} ]]; then
    MOUNT_DIR="/home/${USERNAME}/projects/${REPO}"
#add support for Anne-Michelle repo location
elif [[ ! -d ${MOUNT_DIR} ]]; then
    MOUNT_DIR="/home/${USERNAME}/School/662ENPM/${REPO}"
#end add
fi
if [[ -d ${MOUNT_DIR} ]]; then
    PARAMS="${PARAMS} -v ${MOUNT_DIR}:/mnt/${REPO}"
else
    echo "Repository not found.  Try keeping it in ~/${REPO}, ~/projects/${REPO}, or ~/cpp/${REPO} if you want it mounted to the container."
fi

# assume sudo if not on WSL
if [[ -e /etc/wsl.conf ]]; then
    docker run --user ${USER_ID}:${GROUP_ID} ${PARAMS} -it  --name ${PROJECT}${1} --hostname ${PROJECT}${1} ${PROJECT}
else
    sudo docker run --user ${USER_ID}:${GROUP_ID} ${PARAMS} -it  --name ${PROJECT}${1} --hostname ${PROJECT}${1} ${PROJECT}
fi

