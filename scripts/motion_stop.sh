#!/bin/bash


# 이름에 loco_bridge 또는 walking_policy_node가 포함된 모든 프로세스 강제 종료
pkill -9 -f "motion.launch.py"
pkill -9 -f "loco_bridge"
pkill -9 -f "walking_policy_node"
pkill -9 -f "joy_node"
