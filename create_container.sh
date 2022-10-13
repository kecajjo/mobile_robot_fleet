#!/bin/bash
scriptDir=$(dirname $0 | xargs -i readlink -f {})
container_name=mobile_robot_fleet
version="0.1"
does_exist=$(docker image ls $container_name:$version | grep -ci1 $container_name)
if [ $does_exist == "0" ] ; then
	docker build -t $container_name:$version .
fi
docker run --rm \
    --privileged \
    --env DISPLAY \
    -v $HOME/.Xauthority:/root/.Xauthority \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$scriptDir/src/:/home/developer/catkin_ws/src" \
    -it $container_name:$version /bin/bash
