#!/bin/bash

# ワークスペースのパス
WS_PATH=/workspaces

# ROS 2 環境を読み込み
source ${WS_PATH}/install/setup.bash

echo "=========================================="
echo "Visual SLAM Launch Configuration"
echo "=========================================="
echo ""
echo "Select mode:"
echo "  1) Mapping mode (create new map)"
echo "  2) Localization mode (use existing map)"
echo ""
read -p "Enter choice [1-2]: " MODE_CHOICE

case $MODE_CHOICE in
  1)
    LAUNCH_GLOBAL_LOC="false"
    MODE_NAME="Mapping"
    SOURCE_DIR="/workspaces/record/map_source"
    ;;
  2)
    LAUNCH_GLOBAL_LOC="true"
    MODE_NAME="Localization"
    SOURCE_DIR="/workspaces/src/launch/localization_launch/map"
    ;;
  *)
    echo "Invalid choice. Exiting."
    exit 1
    ;;
esac

echo ""
echo "Selected: $MODE_NAME mode"
echo ""

# Base directories
MAP_BASE_DIR="/workspaces/src/launch/localization_launch/map"
KEYFRAMES_BASE_DIR="/workspaces/src/launch/localization_launch/keyframes"

# Get available directories
if [ -d "$SOURCE_DIR" ]; then
  mapfile -t AVAILABLE_MAPS < <(ls -1 "$SOURCE_DIR" 2>/dev/null | sort -r)
fi

# Show available maps and let user select
if [ ${#AVAILABLE_MAPS[@]} -gt 0 ]; then
  echo "Available maps in $SOURCE_DIR:"
  echo ""
  for i in "${!AVAILABLE_MAPS[@]}"; do
    echo "  $((i+1))) ${AVAILABLE_MAPS[$i]}"
  done
  echo "  0) Enter custom name"
  echo ""
  
  read -p "Select map [0-${#AVAILABLE_MAPS[@]}]: " MAP_CHOICE
  
  if [ "$MAP_CHOICE" = "0" ]; then
    # Custom name
    DEFAULT_MAP_NAME=$(date +%Y%m%d_%H%M%S)
    echo ""
    echo "Default map name: $DEFAULT_MAP_NAME"
    read -p "Enter map name (press Enter to use default): " USER_MAP_NAME
    
    if [ -z "$USER_MAP_NAME" ]; then
      MAP_NAME="$DEFAULT_MAP_NAME"
    else
      MAP_NAME="$USER_MAP_NAME"
    fi
  elif [ "$MAP_CHOICE" -ge 1 ] && [ "$MAP_CHOICE" -le ${#AVAILABLE_MAPS[@]} ]; then
    # Selected from list
    MAP_NAME="${AVAILABLE_MAPS[$((MAP_CHOICE-1))]}"
  else
    echo "Invalid selection. Exiting."
    exit 1
  fi
else
  # No maps available, ask for custom name
  echo "No existing maps found in $SOURCE_DIR"
  echo ""
  DEFAULT_MAP_NAME=$(date +%Y%m%d_%H%M%S)
  echo "Default map name: $DEFAULT_MAP_NAME"
  read -p "Enter map name (press Enter to use default): " USER_MAP_NAME
  
  if [ -z "$USER_MAP_NAME" ]; then
    MAP_NAME="$DEFAULT_MAP_NAME"
  else
    MAP_NAME="$USER_MAP_NAME"
  fi
fi

echo ""
echo "Using map name: $MAP_NAME"

# Set paths based on mode and build launch arguments
LAUNCH_ARGS="launch_global_localization:=$LAUNCH_GLOBAL_LOC"

if [ "$LAUNCH_GLOBAL_LOC" = "false" ]; then
  # Mapping mode - check if source data exists
  SOURCE_DATA_PATH="$SOURCE_DIR/$MAP_NAME"
  
  if [ ! -d "$SOURCE_DATA_PATH" ]; then
    echo ""
    echo "ERROR: Source data directory does not exist: $SOURCE_DATA_PATH"
    echo "Please record rosbag data first using the recording script."
    exit 1
  fi
  
  # Check if rosbag exists
  if [ ! -f "$SOURCE_DATA_PATH/camera_pose_0.mcap" ] && [ -z "$(find "$SOURCE_DATA_PATH" -name "*.mcap" 2>/dev/null)" ]; then
    echo ""
    echo "WARNING: No .mcap files found in $SOURCE_DATA_PATH"
    echo "Make sure rosbag data has been recorded."
    read -p "Continue anyway? [y/N]: " CONTINUE
    if [[ ! "$CONTINUE" =~ ^[Yy]$ ]]; then
      exit 1
    fi
  fi
  
  SAVE_MAP_PATH="$MAP_BASE_DIR/$MAP_NAME"
  
  # Create map directory if it doesn't exist
  if [ ! -d "$SAVE_MAP_PATH" ]; then
    echo ""
    echo "Creating map directory: $SAVE_MAP_PATH"
    mkdir -p "$SAVE_MAP_PATH"
    
    if [ $? -ne 0 ]; then
      echo "ERROR: Failed to create map directory"
      exit 1
    fi
  else
    echo ""
    echo "WARNING: Map directory already exists: $SAVE_MAP_PATH"
    read -p "Overwrite existing map? [y/N]: " OVERWRITE
    if [[ ! "$OVERWRITE" =~ ^[Yy]$ ]]; then
      echo "Cancelled."
      exit 1
    fi
  fi
  
  LAUNCH_ARGS="$LAUNCH_ARGS save_map_path:=$SAVE_MAP_PATH"
  
  echo ""
  echo "=========================================="
  echo "Launch Configuration:"
  echo "  Mode: $MODE_NAME"
  echo "  Map name: $MAP_NAME"
  echo "  Source rosbag: $SOURCE_DATA_PATH"
  echo "  Save map to: $SAVE_MAP_PATH"
  echo "=========================================="
else
  # Localization mode - validate map and keyframes exist
  LOAD_MAP_PATH="$MAP_BASE_DIR/$MAP_NAME"
  VGL_MAP_DIR="$KEYFRAMES_BASE_DIR/$MAP_NAME"
  
  # Validation
  if [ ! -d "$LOAD_MAP_PATH" ]; then
    echo ""
    echo "ERROR: Map directory does not exist: $LOAD_MAP_PATH"
    echo "Please create the map first using mapping mode."
    exit 1
  fi
  
  if [ ! -d "$VGL_MAP_DIR" ]; then
    echo ""
    echo "ERROR: Keyframes directory does not exist: $VGL_MAP_DIR"
    echo "Please generate keyframes for this map."
    exit 1
  fi
  
  LAUNCH_ARGS="$LAUNCH_ARGS load_map_path:=$LOAD_MAP_PATH vgl_map_dir:=$VGL_MAP_DIR"
  
  echo ""
  echo "=========================================="
  echo "Launch Configuration:"
  echo "  Mode: $MODE_NAME"
  echo "  Map name: $MAP_NAME"
  echo "  Load map from: $LOAD_MAP_PATH"
  echo "  Keyframes from: $VGL_MAP_DIR"
  echo "=========================================="
fi

echo ""
echo "Command to be executed:"
echo "  ros2 launch localization_launch vslam.launch.xml $LAUNCH_ARGS"
echo ""
read -p "Press Enter to launch, or Ctrl+C to cancel..."

# Launch with appropriate parameters
ros2 launch localization_launch vslam.launch.xml $LAUNCH_ARGS