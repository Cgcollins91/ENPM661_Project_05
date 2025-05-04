sudo docker run --rm --net=host -e DISPLAY=$DISPLAY \
           -v /tmp/.X11-unix:/tmp/.X11-unix \
           -v /home/cgcollins91/projects/ENPM662_Project_05:/mnt/ENPM662_Project_05 \
           -v /dev/shm:/dev/shm \
           -it humblebot


cd mnt/ENPM662_Project_05/project2