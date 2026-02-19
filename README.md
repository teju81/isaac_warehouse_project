# isaac_warehouse_project

## Isaac Sim 5.1 Setup

- Download the Isaac Sim 5.1 zip file and perform quick install as given in the [Link](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/quick-install.html)
- Download the full assets zip file, extract and link it to Isaac Sim

**Note 1: How to link assets folder to Isaac Sim**
Assuming the installation folder is ```~/isaac-sim/isaac-sim-standalone-5.1.0-linux-x86_64```
Edit the file in ```apps/isaacsim.exp.base.kit``` 
- Add the text under the ```[settings]``` tag at the end
  ```
    persistent.isaac.asset_root.default = "/media/raviteja/Ubuntu_Data2/isaac-sim-assets/Assets/Isaac/5.1"

    exts."isaacsim.asset.browser".folders = [
        "/media/raviteja/Ubuntu_Data2/isaac-sim-assets/Assets/Isaac/5.1/Isaac/Robots",
        "/media/raviteja/Ubuntu_Data2/isaac-sim-assets/Assets/Isaac/5.1/Isaac/Environments",
        "/media/raviteja/Ubuntu_Data2/isaac-sim-assets/Assets/Isaac/5.1/Isaac/Props",
        "/media/raviteja/Ubuntu_Data2/isaac-sim-assets/Assets/Isaac/5.1/Isaac/Materials",
        "/media/raviteja/Ubuntu_Data2/isaac-sim-assets/Assets/Isaac/5.1/Isaac/Sensors",
        "/media/raviteja/Ubuntu_Data2/isaac-sim-assets/Assets/Isaac/5.1/Isaac/People",
        "/media/raviteja/Ubuntu_Data2/isaac-sim-assets/Assets/Isaac/5.1/Isaac/IsaacLab",
        "/media/raviteja/Ubuntu_Data2/isaac-sim-assets/Assets/Isaac/5.1/Isaac/Samples",
    ]
  ```
  
## Launching Isaac Sim 5.1 Environment with Python
1. Launching Simulation (Host side)
  ```
    cd ~/isaac-sim/isaac-sim-standalone-5.1.0-linux-x86_64

    # Set DDS env for ROS 2 communication with Docker container
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
    export ROS_DOMAIN_ID=47
   ./python.sh ~/mycode/isaac_warehouse_project/spawn_warehouse.py
  ```
2. Joystick Teleop (Docker container)
- Launch the ROS 2 Docker container [Terminal 1]
```
docker run -it --rm --network host \
  --device=/dev/input \
  -v /home/raviteja/mycode:/root/mycode \
  <docker_image_with_ros2> bash
```
- Inside the container:

```
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_DOMAIN_ID=47
source /opt/ros/humble/setup.bash

# start the joystick driver
ros2 run joy joy_node
```
- [Terminal 2] Open a second terminal into the same container ```docker exec -it <container_id> bash```, set the same exports, then:
```
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
export ROS_DOMAIN_ID=47
source /opt/ros/humble/setup.bash

# Teleop Carter (left stick = linear, right stick = angular)
python3 /root/mycode/isaac_warehouse_project/joy_teleop.py \
  --topic /carter/cmd_vel \
  --linear-axis 1 --angular-axis 2

# Or teleop the quadcopter
python3 /root/mycode/isaac_warehouse_project/joy_teleop.py \
  --topic /quadcopter/cmd_vel \
  --linear-axis 1 --angular-axis 2 --vertical-axis 3

# Or teleop the explorer camera
python3 /root/mycode/isaac_warehouse_project/joy_teleop.py \
  --topic /explorer/cmd_vel \
  --linear-axis 1 --angular-axis 2 --vertical-axis 3
```
Remember: press the Mode button on the Logitech RumblePad 2 so the LED is on (enables analog sticks).
