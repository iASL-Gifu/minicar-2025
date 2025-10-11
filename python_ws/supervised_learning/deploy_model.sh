#!/bin/bash

# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

# This script converts a local ONNX model to a TensorRT engine for deployment
# with Triton inside the Isaac ROS environment.

# === Default arguments for PilotNet model ===
INPUT_ONNX_PATH="" # Will be set by a required command-line argument
MODEL_FILE_NAME="pilotnet.onnx"
HEIGHT="120"
WIDTH="160"
CONFIG_FILE="pilotnet_config.pbtxt"
PRECISION="fp16"
MAX_BATCH_SIZE="1"

# --- Functions ---

function print_parameters() {
  echo
  echo "***************************"
  echo "Using parameters for PilotNet:"
  echo "INPUT_ONNX_PATH : $INPUT_ONNX_PATH"
  echo "MODEL_FILE_NAME : $MODEL_FILE_NAME"
  echo "INPUT_SHAPE     : 3x${HEIGHT}x${WIDTH}"
  echo "PRECISION       : $PRECISION"
  echo "MAX_BATCH_SIZE  : $MAX_BATCH_SIZE"
  echo "CONFIG_FILE     : $CONFIG_FILE"
  echo "***************************"
  echo
}

function setup_model() {
  # Check if the input ONNX file exists
  if [[ ! -f "$INPUT_ONNX_PATH" ]]; then
    echo "Error: Input ONNX file not found at '${INPUT_ONNX_PATH}'"
    exit 1
  fi

  # Set the output directory for the model assets
  local model_name="pilotnet"
  local output_path="${ISAAC_ROS_WS}/isaac_ros_assets/models/${model_name}"
  
  echo "Creating directory: ${output_path}/1"
  rm -rf "${output_path}"
  mkdir -p "${output_path}/1"

  # Copy the local ONNX model to the destination directory
  echo "Copying ONNX model from ${INPUT_ONNX_PATH}"
  cp "${INPUT_ONNX_PATH}" "${output_path}/1/${MODEL_FILE_NAME}"

  echo "Converting ONNX model to a TensorRT Engine Plan (.plan)..."

  # Use trtexec to convert ONNX to a TensorRT engine
  /usr/src/tensorrt/bin/trtexec \
    --onnx="${output_path}/1/${MODEL_FILE_NAME}" \
    --saveEngine="${output_path}/1/model.plan" \
    --minShapes=input_1:1x3x${HEIGHT}x${WIDTH} \
    --optShapes=input_1:1x3x${HEIGHT}x${WIDTH} \
    --maxShapes=input_1:${MAX_BATCH_SIZE}x3x${HEIGHT}x${WIDTH} \
    --${PRECISION} \
    --verbose
  
  # Copy the Triton configuration file
  echo "Copying config.pbtxt to ${output_path}"
  local pkg_share_path=$(ros2 pkg prefix isaac_ros_e2e_pilot --share)
  cp "${pkg_share_path}/config/${CONFIG_FILE}" \
    "${output_path}/config.pbtxt"
    
  echo "✅ Completed PilotNet model setup."
}

function show_help() {
  echo "Usage: $0 -i /path/to/your/model.onnx [options]"
  echo "Options:"
  echo "  -i, --input-onnx        [REQUIRED] Path to the local ONNX model file."
  echo "  -c, --config-file       Name of the pbtxt config file in the config dir. (Default: ${CONFIG_FILE})"
  echo "  -p, --precision         Precision for TensorRT engine (fp32, fp16, int8). (Default: ${PRECISION})"
  echo "  -b, --max-batch-size    Maximum batch size for the TensorRT engine. (Default: ${MAX_BATCH_SIZE})"
  echo "  -h, --help              Show this help message."
}

# --- Main script execution ---

# Parse command line arguments
OPTIONS=i:c:p:b:h
LONGOPTS=input-onnx:,config-file:,precision:,max-batch-size:,help

PARSED=$(getopt --options=$OPTIONS --longoptions=$LONGOPTS --name "$0" -- "$@")
if [[ $? -ne 0 ]]; then
    exit 1
fi
eval set -- "$PARSED"

while true; do
    case "$1" in
        -i|--input-onnx)
          INPUT_ONNX_PATH="$2"
          shift 2
          ;;
        -c|--config-file)
          CONFIG_FILE="$2"
          shift 2
          ;;
        -p|--precision)
          PRECISION="$2"
          shift 2
          ;;
        -b|--max-batch-size)
          MAX_BATCH_SIZE="$2"
          shift 2
          ;;
        -h|--help)
          show_help
          exit 0
          ;;
        --)
          shift
          break
          ;;
        *)
          echo "Unknown argument: $1"
          show_help
          exit 1
          ;;
    ac
done

# Check if required argument is provided
if [[ -z "$INPUT_ONNX_PATH" ]]; then
  echo "Error: --input-onnx is a required argument."
  show_help
  exit 1
fi

# Print parameters and run the setup
print_parameters
setup_model