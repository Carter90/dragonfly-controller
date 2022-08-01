#!/bin/bash

docker network inspect ros-net >/dev/null 2>&1 || \
    docker network create ros-net --subnet=172.18.0.0/16

docker run -it \
    --network ros-net \
    dragonfly-controller:latest \
    /bin/sh -c 'for i in {1..3}
    do
      ros2 run dragonfly announce dragonfly$i &
      ros2 run dragonfly command dragonfly$i &
      ros2 run dragonfly virtualco2publisher dragonfly$i &
    done;
    ros2 run dragonfly announce dragonfly4 &
    ros2 run dragonfly command dragonfly4 &
    ros2 run dragonfly virtualco2publisher dragonfly4'
    
