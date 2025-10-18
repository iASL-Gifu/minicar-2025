#!/bin/bash

# Create CUVGL Map Script
# This script creates a global localization map with bow index and vocabulary

# Base directory
BASE_DIR="/workspaces/src/launch/localization_launch/keyframes"

# Check if base directory exists
if [ ! -d "$BASE_DIR" ]; then
    echo "Error: Base directory does not exist: $BASE_DIR"
    exit 1
fi

# Get list of subdirectories
mapfolders=($(find "$BASE_DIR" -maxdepth 1 -type d ! -name "keyframes" | sort))

# Check if any subdirectories exist
if [ ${#mapfolders[@]} -eq 0 ]; then
    echo "Error: No subdirectories found in $BASE_DIR"
    exit 1
fi

# Display available map folders
echo "============================================"
echo "Create CUVGL Map"
echo "============================================"
echo ""
echo "Available map folders:"
echo ""

for i in "${!mapfolders[@]}"; do
    folder_name=$(basename "${mapfolders[$i]}")
    echo "  $((i+1))) $folder_name"
done

echo ""
read -p "Select map folder number: " selection

# Validate selection
if ! [[ "$selection" =~ ^[0-9]+$ ]] || [ "$selection" -lt 1 ] || [ "$selection" -gt ${#mapfolders[@]} ]; then
    echo "Error: Invalid selection"
    exit 1
fi

# Get selected map folder
MAP_FOLDER="${mapfolders[$((selection-1))]}"
folder_name=$(basename "$MAP_FOLDER")

echo ""
echo "Selected: $folder_name"
echo "Path: $MAP_FOLDER"
echo "Starting CUVGL map creation..."
echo ""

# Run the ROS2 command
ros2 run isaac_mapping_ros create_cuvgl_map.py --map_folder="$MAP_FOLDER" --no-extract_feature

# Check if command was successful
if [ $? -eq 0 ]; then
    echo ""
    echo "============================================"
    echo "✓ CUVGL map creation completed successfully!"
    echo "Map: $folder_name"
    echo "============================================"
else
    echo ""
    echo "============================================"
    echo "✗ CUVGL map creation failed!"
    echo "============================================"
    exit 1
fi