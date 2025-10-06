# Minicar 2025

## setup

## setup for ISAAC ROS
```bash
## jetson clocks
sudo /usr/bin/jetson_clocks
## power mode
sudo /usr/sbin/nvpmodel -m 0
## docker user
sudo usermod -aG docker $USER
newgrp docker

## workspaceの作成
mkdir -p "${HOME}/workspace/"
git clone 
echo "export ISAAC_ROS_WS=${HOME}/workspace/minicar-2025/ros2_ws/src/isaac-ros/" >> ~/.bashrc
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