#!/bin/bash

# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may
# obtain a copy of the License at
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

# === Default arguments ===
INPUT_ONNX_PATH="" 
MODEL_NAME=""      
HEIGHT="120"
WIDTH="160"
INPUT_TENSOR_NAME="input_1"
CONFIG_FILE="pilotnet_config.pbtxt"
PRECISION="fp16"
MAX_BATCH_SIZE="1"

# --- Functions ---

function print_parameters() {
  echo
  echo "***************************"
  echo "Using the following parameters:"
  echo "MODEL_NAME        : $MODEL_NAME"
  echo "INPUT_ONNX_PATH   : $INPUT_ONNX_PATH"
  echo "INPUT_TENSOR_NAME : $INPUT_TENSOR_NAME"
  echo "INPUT_SHAPE       : 3x${HEIGHT}x${WIDTH}"
  echo "PRECISION         : $PRECISION"
  echo "MAX_BATCH_SIZE    : $MAX_BATCH_SIZE"
  echo "CONFIG_FILE       : $CONFIG_FILE"
  echo "***************************"
  echo
}

function setup_model() {
  # Check if the input ONNX file exists
  if [[ ! -f "$INPUT_ONNX_PATH" ]]; then
    echo "❌ Error: Input ONNX file not found at '${INPUT_ONNX_PATH}'"
    exit 1
  fi

  # Set the output directory for the model assets
  local output_path="${ISAAC_ROS_WS}/isaac_ros_assets/models/${MODEL_NAME}"
  
  # Find the next available version number to avoid overwriting
  local version=1
  while [[ -d "${output_path}/${version}" ]]; do
    version=$((version + 1))
  done
  echo "✅ A new version will be created at: ${output_path}/${version}"
  
  local version_path="${output_path}/${version}"
  echo "Creating directory: ${version_path}"
  mkdir -p "${version_path}"

  # Get the model file name from the input path
  local model_file_name=$(basename "$INPUT_ONNX_PATH")

  # Copy the local ONNX model to the destination directory
  echo "Copying ONNX model from ${INPUT_ONNX_PATH}"
  cp "${INPUT_ONNX_PATH}" "${version_path}/${model_file_name}"

  echo "Converting ONNX model to a TensorRT Engine Plan (.plan)..."

  # Use trtexec to convert ONNX to a TensorRT engine
  /usr/src/tensorrt/bin/trtexec \
    --onnx="${version_path}/${model_file_name}" \
    --saveEngine="${version_path}/model.plan" \
    --minShapes=${INPUT_TENSOR_NAME}:1x3x${HEIGHT}x${WIDTH} \
    --optShapes=${INPUT_TENSOR_NAME}:1x3x${HEIGHT}x${WIDTH} \
    --maxShapes=${INPUT_TENSOR_NAME}:${MAX_BATCH_SIZE}x3x${HEIGHT}x${WIDTH} \
    --${PRECISION} \
    --verbose
  
  # Copy the Triton configuration file
  # Note: You might need to create different config files for different models
  echo "Copying config.pbtxt to ${output_path}"
  local pkg_share_path=$(ros2 pkg prefix isaac_ros_e2e_pilot --share)
  cp "${pkg_share_path}/config/${CONFIG_FILE}" \
    "${output_path}/config.pbtxt"
    
  echo "✅ Completed model setup for ${MODEL_NAME} (version ${version})."
}

function show_help() {
  echo "Usage: $0 -i /path/to/model.onnx -m my_model_name [options]"
  echo "Options:"
  echo "  -i, --input-onnx        [REQUIRED] Path to the local ONNX model file."
  echo "  -m, --model-name        [REQUIRED] Name for the model in the model repository."
  echo "  -c, --config-file       Name of the pbtxt config file in the config dir. (Default: ${CONFIG_FILE})"
  echo "  -p, --precision         Precision for TensorRT engine (fp32, fp16, int8). (Default: ${PRECISION})"
  echo "  -b, --max-batch-size    Maximum batch size for the TensorRT engine. (Default: ${MAX_BATCH_SIZE})"
  echo "      --height            Input image height. (Default: ${HEIGHT})"
  echo "      --width             Input image width. (Default: ${WIDTH})"
  echo "      --input-name        Name of the input tensor in the ONNX model. (Default: ${INPUT_TENSOR_NAME})"
  echo "  -h, --help              Show this help message."
}

# --- Main script execution ---

# Parse command line arguments
OPTIONS=i:m:c:p:b:h
LONGOPTS=input-onnx:,model-name:,config-file:,precision:,max-batch-size:,height:,width:,input-name:,help

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
        -m|--model-name)
          MODEL_NAME="$2"
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
        --height)
          HEIGHT="$2"
          shift 2
          ;;
        --width)
          WIDTH="$2"
          shift 2
          ;;
        --input-name)
          INPUT_TENSOR_NAME="$2"
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
    esac
done

# Check if required arguments are provided
if [[ -z "$INPUT_ONNX_PATH" ]] || [[ -z "$MODEL_NAME" ]]; then
  echo "❌ Error: --input-onnx and --model-name are required arguments."
  show_help
  exit 1
fi

# Print parameters and run the setup
print_parameters
setup_model