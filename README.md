# Minicar 2025

## setup

## 1. base
```bash
mkdir -p "${HOME}/workspace/"
cd workspace
git clone https://github.com/iASL-Gifu/minicar-2025.git 
cd minicar-2025
vcs import < packages.repos
```

## 2. setup for jetracer
```bash
sudo apt install python3-pip
sudo pip3 install -U jetson-stats

mkdir lib
cd lib 
git clone https://github.com/NVIDIA-AI-IOT/jetracer.git
cd jetracer
sudo python3 setup.py install
pip3 install traitlets questionary
```

## setup for ISAAC ROS on Jetson Orin Nano 8GB
```bash
## jetson clocks
sudo /usr/bin/jetson_clocks
## power mode MAX
sudo /usr/sbin/nvpmodel -m 2
## docker user
sudo usermod -aG docker $USER
newgrp docker
sudo systemctl daemon-reload && sudo systemctl restart docker
sudo apt-get install git-lfs
git lfs install --skip-repo

## workspaceの設定
echo "export ISAAC_ROS_WS=${HOME}/workspace/minicar-2025/ros2_ws/src/isaac_ros" >> ~/.bashrc
source ~/.bashrc

## VPI(Vision Programming Interface)
sudo nvidia-ctk cdi generate --mode=csv --output=/etc/cdi/nvidia.yaml

# Add Jetson public APT repository
sudo apt-get update
sudo apt-get install software-properties-common
sudo apt-key adv --fetch-key https://repo.download.nvidia.com/jetson/jetson-ota-public.asc
sudo add-apt-repository 'deb https://repo.download.nvidia.com/jetson/common r36.4 main'
sudo apt-get update
sudo apt-get install -y pva-allow-2
```
## setup for DL(Deep Learning)
```bash
## CUDA 12.6
sudo apt install cuda-toolkit-12-6

UNDER_CONSTRUCTION ...
```

## scripts

## hotspot
```bash
bash hotspot.sh wlan0 tamiya22 tamiya22
```

## bluetooth
```bash
## A0:AB:51:5F:62:86にbluetooth接続をする
bash bluetooth.sh <MAC_ADDRESS>
```

## tmux
```bash
bash tmux.sh <session_name>
```

## docker run
```bash
## setting
cd ${ISAAC_ROS_WS}/src/isaac_ros_common/scripts && \
cat > .isaac_ros_common-config << EOF
CONFIG_IMAGE_KEY=ros2_humble.realsense.cyclone_dds.isaac_ros_preset
CONFIG_DOCKER_SEARCH_DIRS=("../docker/")
EOF

## run
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
./scripts/run_dev.sh -d ${ISAAC_ROS_WS}
```

# run

## visual slam

### 1. terminal 1 実行
```bash
## docker run
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
./scripts/run_dev.sh -d ${ISAAC_ROS_WS}

sudo apt-get update
sudo apt-get install -y ros-humble-isaac-ros-examples ros-humble-isaac-ros-realsense
rosdep update && rosdep install --from-paths ${ISAAC_ROS_WS}/src/isaac_ros_visual_slam/isaac_ros_visual_slam --ignore-src -y

## build
cd ${ISAAC_ROS_WS}/ && \
   colcon build --symlink-install --packages-up-to isaac_ros_visual_slam --base-paths ${ISAAC_ROS_WS}/src/isaac_ros_visual_slam/isaac_ros_visual_slam

source install/setup.bash
ros2 launch isaac_ros_examples isaac_ros_examples.launch.py launch_fragments:=realsense_stereo_rect,visual_slam \
interface_specs_file:=${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_visual_slam/quickstart_interface_specs.json \
base_frame:=camera_link camera_optical_frames:="['camera_infra1_optical_frame', 'camera_infra2_optical_frame']"
```

### 2. terminal 2 可視化
```bash
## docker run
cd ${ISAAC_ROS_WS}/src/isaac_ros_common && \
./scripts/run_dev.sh -d ${ISAAC_ROS_WS}

rviz2 -d $(ros2 pkg prefix isaac_ros_visual_slam --share)/rviz/default.cfg.rviz
```