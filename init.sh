#!/bin/bash

BASE_PATH=$(pwd)
RNG=$(tr -dc '0-9' < /dev/urandom | head -c 6)
cd ${BASE_PATH}/docker/humblebot
${BASE_PATH}/docker/humblebot/build.sh
${BASE_PATH}/docker/humblebot/launch.sh ${RNG}
pwd

