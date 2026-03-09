#!/bin/bash

source ~/Workspace/Booster_K1_3v3_Demo/install/setup.bash

cd `dirname $0`
cd ..

./scripts/motion_stop.sh

source ./install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=./configs/fastdds.xml

nohup ros2 launch motion motion.launch.py ns:=robot1 > /dev/null 2>&1 &
nohup ros2 launch motion motion.launch.py ns:=robot2 > /dev/null 2>&1 &
nohup ros2 launch motion motion.launch.py ns:=robot3 > /dev/null 2>&1 &
