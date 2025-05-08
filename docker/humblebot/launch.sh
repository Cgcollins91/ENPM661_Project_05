#!/bin/bash

# Project: ENPM662-Project1-Group1
# Description: Launch a Container

USERNAME=$(id -un)
USER_ID=$(id -u)
GROUP_ID=$(id -g)
REPO="ENPM662_Project_05"
PROJECT='humblebot'
X11='/tmp/.X11-unix:/tmp/.X11-unix'
WSLG='/mnt/wslg:/mnt/wslg'
PARAMS="-v ${X11}"
TARGET=/mnt/${REPO}

if [[ "${USERNAME}" == "root" ]]; then
    echo -e "This script is not intended to be run as root or with sudo.\n"
    exit 1
fi

# if WSL environment, fixup graphics
if [[ -e /etc/wsl.conf ]]; then
    PARAMS="${PARAMS} -v ${WSLG}"
fi

# try to find project directory and mount to container
MOUNT_DIR="/home/${USERNAME}/projects/${REPO}"
if [[ ! -d ${MOUNT_DIR} ]]; then
    MOUNT_DIR="/home/${USERNAME}/${REPO}"
fi
if [[ ! -d ${MOUNT_DIR} ]]; then
    MOUNT_DIR="/home/${USERNAME}/projects/${REPO}"
fi
if [[ ! -d ${MOUNT_DIR} ]]; then
    MOUNT_DIR="/home/${USERNAME}/umd/${REPO}"
fi
if [[ ! -d ${MOUNT_DIR} ]]; then
    MOUNT_DIR="/home/${USERNAME}/School/662ENPM/${REPO}"
fi
if [[ -d ${MOUNT_DIR} ]]; then
    PARAMS="${PARAMS} -v ${MOUNT_DIR}:${TARGET}"
else
    echo "Repository not found.  Try keeping it in ~/${REPO}, ~/projects/${REPO}, or ~/cpp/${REPO} if you want it mounted to the container."
fi



# assume sudo if not on WSL
if [[ -e /etc/wsl.conf ]]; then
    docker run --user ${USER_ID}:${GROUP_ID} ${PARAMS} -it  --name ${PROJECT}${1} --hostname ${PROJECT}${1} ${PROJECT}
else
    echo "HOST repo dir : ${MOUNT_DIR}"
    echo "Docker -v arg : ${PARAMS}"
    sudo docker run --user ${USER_ID}:${GROUP_ID} ${PARAMS} -it  --name ${PROJECT}${1} --hostname ${PROJECT}${1} ${PROJECT}
fi

