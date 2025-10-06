# Minicar 2025

## setup

## 1. base
```bash
mkdir -p "${HOME}/workspace/"
git clone https://github.com/iASL-Gifu/minicar-2025.git
cd minicar-2025
vcs import < packages.repos
```

## 2. setup for jetracer
```bash
sudo apt install python3-pip
sudo pip3 install -U jetson-stats

cd lib/jetracer
sudo python3 setup.py install
pip3 install traitlets questionary
```

## setup for ISAAC ROS on Jetson Orin Nano 8GB
```bash
## jetson clocks
sudo /usr/bin/jetson_clocks
## power mode
sudo /usr/sbin/nvpmodel -m 0
## docker user
sudo usermod -aG docker $USER
newgrp docker

## workspaceの設定
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