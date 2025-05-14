#!/bin/bash
# start docker container with xorg windows and more

xhost +local:$USERNAME
export ROS_DOMAIN_ID=0

if [ "$(docker ps -aq -f name=airos)" ]
then
  if ["$(docker ps -q -f name=airos)" ]
  then
    echo "container is running. attaching..."
  else
    echo "starting existing container"
    docker start airos
  fi
  docker exec -it \
      -e DISPLAY=$DISPLAY \
      -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
      airos bash
else
      # --volume="$(pwd):/home/rosuser/ros2_ws" \
  docker run -it \
      --name airos \
      --env DISPLAY=$DISPLAY \
      --env ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
      --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
      --volume="$HOME/.ssh:/home/ros/.ssh:ro" \
      --volume="$HOME/.gitconfig:/home/ros/.gitconfig:ro" \
      --device=/dev/dri \
      --group-add video \
      --net=host \
      --privileged \
      ros2 bash
fi


