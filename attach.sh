#!/bin/bash
docker ps -a
id=`docker ps -a |head -2 |tail -1 |awk '{print $NF}'`
echo "Attempting to connect to ${id}"
docker start ${id}
docker exec -it ${id} bash

