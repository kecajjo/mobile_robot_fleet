#!/bin/bash
scriptDir=$(dirname $0 | xargs -i readlink -f {})
repo_root=$(readlink -f "$scriptDir/..")
image_name="mobile_robot_fleet"
container_name=$image_name
version="0.3"

function print_banner() {
    echo ">> Image version to be used: $version"
    echo ">> Mounting host directory \"$repo_root\" under \"/mobile_robot_fleet\" share.

    To enter the container type:

      $ docker exec -it $container_name /bin/bash

    To build the workspace inside container type:

      $ cd catkin_ws/
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
	docker build \
      -t $image_name:$version . \
      -f "$repo_root/Dockerfile"
fi

print_banner
docker run \
    --name "$container_name" \
    --privileged \
    --env DISPLAY \
    -v $HOME/.Xauthority:/root/.Xauthority \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v "$repo_root/:/home/developer/mobile_robot_fleet" \
    -itd $image_name:$version /bin/bash
