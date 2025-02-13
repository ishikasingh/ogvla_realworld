#!/bin/bash

source .env
source /home/abrar/hsc/Allegro-Hand-Controller-DIME/devel/setup.bash

python run_server.py \
    arm_hand_type=franka_allegro