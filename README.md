# About

## Commands
```bash
# Launches the control node with fake hardware
ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=px100 use_sim:=true

python3 data_generator.py --robot_name=px100 --robot_model=px100
```