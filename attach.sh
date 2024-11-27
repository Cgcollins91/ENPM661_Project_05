#!/bin/bash
id=`docker ps -a |head -2 |tail -1 |awk '{print $13}'`
docker start ${id}
docker exec -it ${id} bash

