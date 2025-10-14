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

## 2. setup for ISAAC ROS on Jetson Orin Nano 8GB
```bash
sudo /usr/bin/jetson_clocks
sudo /usr/sbin/nvpmodel -m 2

sudo usermod -aG docker $USER
newgrp docker
sudo systemctl daemon-reload && sudo systemctl restart docker

sudo apt-get install git-lfs
git lfs install --skip-repo

echo "export ISAAC_ROS_WS=${HOME}/workspace/minicar-2025/ros2_ws/" >> ~/.bashrc
source ~/.bashrc

sudo nvidia-ctk cdi generate --mode=csv --output=/etc/cdi/nvidia.yaml

sudo apt-get update
sudo apt-get install software-properties-common
sudo apt-key adv --fetch-key https://repo.download.nvidia.com/jetson/jetson-ota-public.asc
sudo add-apt-repository 'deb https://repo.download.nvidia.com/jetson/common r36.4 main'
sudo apt-get update
sudo apt-get install -y pva-allow-2
```
## setup for DL(Deep Learning)
```bash
sudo apt install cuda-toolkit-12-6
pip install torch==2.8.0 torchvision==0.23.0  --index-url=https://pypi.jetson-ai-lab.io/jp6/cu126
```

## setup for ROS2
```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

sudo apt update && sudo apt install gnupg wget
sudo apt install software-properties-common
sudo add-apt-repository universe

wget -qO - https://isaac.download.nvidia.com/isaac-ros/repos.key | sudo apt-key add -
grep -qxF "deb https://isaac.download.nvidia.com/isaac-ros/release-3 $(lsb_release -cs) release-3.0" /etc/apt/sources.list || \
echo "deb https://isaac.download.nvidia.com/isaac-ros/release-3 $(lsb_release -cs) release-3.0" | sudo tee -a /etc/apt/sources.list
sudo apt-get update

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop

```

## setup for realsense
```bash
git clone https://github.com/jetsonhacks/jetson-orin-librealsense.git
cd jetson-orin-librealsense
tar -xzf install-modules.tar.gz
cd install-modules
sudo ./install-realsense-modules.sh
cd /tmp
rm -rf jetson-orin-librealsense

sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE

sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" 
sudo apt-get update 
sudo apt-get install -y --no-install-recommends librealsense2-utils librealsense2-dev 
sudo apt-get clean 
sudo rm -rf /var/lib/apt/lists/* \
sudo rm -rf /tmp/*

wget https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules
sudo mv 99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

# scripts

## hotspot
```bash
bash hotspot.sh wlan0 tamiya22 tamiya22
```

## bluetooth
```bash
bash bluetooth.sh <MAC_ADDRESS>


## bash bluetooth.sh A0:AB:51:5F:62:86
```

## tmux
```bash
bash tmux.sh <session_name>
```

## download assets
```bash
cd ${ISAAC_ROS_WS}/src
sudo apt-get install -y curl jq tar

NGC_ORG="nvidia"
NGC_TEAM="isaac"
PACKAGE_NAME="isaac_ros_visual_slam"
NGC_RESOURCE="isaac_ros_visual_slam_assets"
NGC_FILENAME="quickstart.tar.gz"
MAJOR_VERSION=3
MINOR_VERSION=2
VERSION_REQ_URL="https://catalog.ngc.nvidia.com/api/resources/versions?orgName=$NGC_ORG&teamName=$NGC_TEAM&name=$NGC_RESOURCE&isPublic=true&pageNumber=0&pageSize=100&sortOrder=CREATED_DATE_DESC"
AVAILABLE_VERSIONS=$(curl -s \
    -H "Accept: application/json" "$VERSION_REQ_URL")
LATEST_VERSION_ID=$(echo $AVAILABLE_VERSIONS | jq -r "
    .recipeVersions[]
    | .versionId as \$v
    | \$v | select(test(\"^\\\\d+\\\\.\\\\d+\\\\.\\\\d+$\"))
    | split(\".\") | {major: .[0]|tonumber, minor: .[1]|tonumber, patch: .[2]|tonumber}
    | select(.major == $MAJOR_VERSION and .minor <= $MINOR_VERSION)
    | \$v
    " | sort -V | tail -n 1
)
if [ -z "$LATEST_VERSION_ID" ]; then
    echo "No corresponding version found for Isaac ROS $MAJOR_VERSION.$MINOR_VERSION"
    echo "Found versions:"
    echo $AVAILABLE_VERSIONS | jq -r '.recipeVersions[].versionId'
else
    mkdir -p ${ISAAC_ROS_WS}/isaac_ros_assets && \
    FILE_REQ_URL="https://api.ngc.nvidia.com/v2/resources/$NGC_ORG/$NGC_TEAM/$NGC_RESOURCE/\
versions/$LATEST_VERSION_ID/files/$NGC_FILENAME" && \
    curl -LO --request GET "${FILE_REQ_URL}" && \
    tar -xf ${NGC_FILENAME} -C ${ISAAC_ROS_WS}/isaac_ros_assets && \
    rm ${NGC_FILENAME}
fi

```
## docker run
```bash
cd ${ISAAC_ROS_WS}/src/isaac_ros/isaac_ros_common/scripts && \
cat > .isaac_ros_common-config << EOF
CONFIG_IMAGE_KEY=ros2_humble.realsense.cyclone_dds.isaac_ros_preset
CONFIG_DOCKER_SEARCH_DIRS=("../docker/")
EOF

cd ${ISAAC_ROS_WS}/src/isaac_ros/isaac_ros_common && \
./scripts/run_dev.sh 
```

# run

## visual slam

### 1. terminal 1 実行
```bash
cd ${ISAAC_ROS_WS}/src/isaac_ros/isaac_ros_common && \
./scripts/run_dev.sh 

source install/setup.bash
ros2 launch isaac_ros_examples isaac_ros_examples.launch.py launch_fragments:=realsense_stereo_rect,visual_slam \
interface_specs_file:=${ISAAC_ROS_WS}/isaac_ros_assets/isaac_ros_visual_slam/quickstart_interface_specs.json \
base_frame:=camera_link camera_optical_frames:="['camera_infra1_optical_frame', 'camera_infra2_optical_frame']"
```

### 2. terminal 2 可視化
```bash
cd ${ISAAC_ROS_WS}/src/isaac_ros/isaac_ros_common && \
./scripts/run_dev.sh 
rviz2 -d $(ros2 pkg prefix isaac_ros_visual_slam --share)/rviz/default.cfg.rviz
```