#!/bin/bash

SCRIPT_PATH=scripts/convert_rosbag_to_csv.py

BAG_PATH=../datasets/px100_configuration_data
FILE_PATH=$BAG_PATH/px100_configuration_data.csv
IDL_PATH=install/arm_controller_msgs/share/arm_controller_msgs/msg/Configuration.idl
TOPIC_NAME=/px100/position_data

python3 $SCRIPT_PATH --bag_path $BAG_PATH --file_path $FILE_PATH --idl_path $IDL_PATH --topic $TOPIC_NAME