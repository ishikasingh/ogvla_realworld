#!/bin/bash
cd /home/abrar/hsc/deoxys_control/deoxys
export LD_LIBRARY_PATH=/home/abrar/hsc/protobuf-3.13.0-install/lib:/usr/local/lib:/home/abrar/hsc/deoxys_control/deoxys:/opt/openrobots/lib:$LD_LIBRARY_PATH
./auto_scripts/auto_gripper.sh /home/abrar/hsc/SeqMultiGraspDeploy/config/deoxys_config_gripper.yaml
