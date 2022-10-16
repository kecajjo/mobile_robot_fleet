#!/bin/bash
scriptDir=$(dirname $0 | xargs -i readlink -f {})
image_name="mobile_robot_fleet"
container_name=$image_name
version="0.1"

function print_banner() {
    echo ">> Image version to be used: $version"
    echo ">> Mounting host directory \"$scriptDir\" under \"/catkin_ws\" share.

    To enter the container type:

      $ docker exec -it $container_name /bin/bash

    To build the workspace inside container type:

      $ catkin build
      "
}

for option in "$@"; do
    case "$option" in
        -n|--name)
            container_name="$2"
            shift 2
            ;;
    esac
done

does_exist=$(docker image ls $image_name:$version | grep -ci1 $image_name)
if [ $does_exist == "0" ] ; then
	docker build -t $image_name:$version .
fi

print_banner
docker run \
    --name "$container_name" \
    --privileged \
    --env DISPLAY \
    -v $HOME/.Xauthority:/root/.Xauthority \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$scriptDir/src/:/home/developer/catkin_ws/src" \
    -itd $image_name:$version /bin/bash