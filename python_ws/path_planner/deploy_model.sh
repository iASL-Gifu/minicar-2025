#!/bin/bash

# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2022 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# (License header as before)
#
# SPDX-License-Identifier: Apache-2.0

# This script converts a local ONNX model to a TensorRT engine for deployment
# with Triton inside the Isaac ROS environment.
#
# --- MODIFIED for Multi-Input models like TrajFormerNano ---

# === Default arguments ===
INPUT_ONNX_PATH="" 
MODEL_NAME=""      
HEIGHT="120"
WIDTH="160"
CONFIG_FILE="trajformer_config.pbtxt" 
PRECISION="fp16"
MAX_BATCH_SIZE="1"

# --- 2つの入力に対応 ---
IMAGE_TENSOR_NAME="input_image" 
ODOM_TENSOR_NAME="input_odoms"
PAST_LEN="5"   # 過去オドメトリのシーケンス長
ODOM_DIM="8"   # 過去オドメトリの特徴量次元

# --- Functions ---

function print_parameters() {
  echo
  echo "***************************"
  echo "Using the following parameters:"
  echo "MODEL_NAME         : $MODEL_NAME"
  echo "INPUT_ONNX_PATH    : $INPUT_ONNX_PATH"
  echo "CONFIG_FILE        : $CONFIG_FILE"
  echo "PRECISION          : $PRECISION"
  echo "MAX_BATCH_SIZE     : $MAX_BATCH_SIZE"
  echo
  echo "--- Input 1 (Image) ---"
  echo "IMAGE_TENSOR_NAME  : $IMAGE_TENSOR_NAME"
  echo "IMAGE_SHAPE        : 3x${HEIGHT}x${WIDTH}"
  echo
  echo "--- Input 2 (Odometry) ---"
  echo "ODOM_TENSOR_NAME   : $ODOM_TENSOR_NAME"
  echo "ODOM_SHAPE         : ${PAST_LEN}x${ODOM_DIM}"
  echo "***************************"
  echo
}

function setup_model() {
  # (Check if input ONNX file exists )
  if [[ ! -f "$INPUT_ONNX_PATH" ]]; then
    echo "❌ Error: Input ONNX file not found at '${INPUT_ONNX_PATH}'"
    exit 1
  fi

  # (Set output path and versioning )
  local output_path="/workspaces/isaac_ros_assets/models/${MODEL_NAME}"
  local version=1
  while [[ -d "${output_path}/${version}" ]]; do
    version=$((version + 1))
  done
  echo "✅ A new version will be created at: ${output_path}/${version}"
  local version_path="${output_path}/${version}"
  echo "Creating directory: ${version_path}"
  mkdir -p "${version_path}"

  # (Copy ONNX model )
  local model_file_name=$(basename "$INPUT_ONNX_PATH")
  echo "Copying ONNX model from ${INPUT_ONNX_PATH}"
  cp "${INPUT_ONNX_PATH}" "${version_path}/${model_file_name}"

  echo "Converting ONNX model to a TensorRT Engine Plan (.plan)..."

  /usr/src/tensorrt/bin/trtexec \
    --onnx="${version_path}/${model_file_name}" \
    --saveEngine="${version_path}/model.plan" \
    --minShapes=${IMAGE_TENSOR_NAME}:1x3x${HEIGHT}x${WIDTH},${ODOM_TENSOR_NAME}:1x${PAST_LEN}x${ODOM_DIM} \
    --optShapes=${IMAGE_TENSOR_NAME}:1x3x${HEIGHT}x${WIDTH},${ODOM_TENSOR_NAME}:1x${PAST_LEN}x${ODOM_DIM} \
    --maxShapes=${IMAGE_TENSOR_NAME}:${MAX_BATCH_SIZE}x3x${HEIGHT}x${WIDTH},${ODOM_TENSOR_NAME}:${MAX_BATCH_SIZE}x${PAST_LEN}x${ODOM_DIM} \
    --${PRECISION} \
    --verbose
  
  echo "Copying config.pbtxt to ${output_path}"
  local pkg_share_path=$(ros2 pkg prefix isaac_ros_e2e_pilot --share)
  
  if [[ ! -f "${pkg_share_path}/config/${CONFIG_FILE}" ]]; then
     echo "⚠️ [WARNING] Config file not found at: ${pkg_share_path}/config/${CONFIG_FILE}"
     echo "    You MUST create this file manually for the model to load in Triton."
  else
     cp "${pkg_share_path}/config/${CONFIG_FILE}" \
       "${output_path}/config.pbtxt"
  fi
    
  echo "✅ Completed model setup for ${MODEL_NAME} (version ${version})."
}

function show_help() {
  echo "Usage: $0 -i /path/to/model.onnx -m my_model_name [options]"
  echo "Converts a 2-input (Image, Odometry) ONNX model to a TensorRT engine."
  echo
  echo "Required Arguments:"
  echo "  -i, --input-onnx        [REQUIRED] Path to the local ONNX model file."
  echo "  -m, --model-name        [REQUIRED] Name for the model in the model repository."
  echo
  echo "Model Shape Arguments:"
  echo "  --height                Input image height. (Default: ${HEIGHT})"
  echo "  --width                 Input image width. (Default: ${WIDTH})"
  echo "  --past-len              Sequence length of past odometry. (Default: ${PAST_LEN})"
  echo "  --odom-dim              Feature dimension of past odometry. (Default: ${ODOM_DIM})"
  echo
  echo "TensorRT & Triton Arguments:"
  echo "  -c, --config-file       Name of the pbtxt config file. (Default: ${CONFIG_FILE})"
  echo "                          (You must create this file for your model!)"
  echo "  -p, --precision         Precision (fp32, fp16, int8). (Default: ${PRECISION})"
  echo "  -b, --max-batch-size    Maximum batch size. (Default: ${MAX_BATCH_SIZE})"
  echo "  --image-tensor-name     Name of the image input tensor. (Default: ${IMAGE_TENSOR_NAME})"
  echo "  --odom-tensor-name      Name of the odometry input tensor. (Default: ${ODOM_TENSOR_NAME})"
  echo "  -h, --help              Show this help message."
}

# --- Main script execution ---

OPTIONS=i:m:c:p:b:h
LONGOPTS=input-onnx:,model-name:,config-file:,precision:,max-batch-size:,help
LONGOPTS+=,height:,width:,past-len:,odom-dim:,image-tensor-name:,odom-tensor-name:

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
        # --- [修正] 形状とテンソル名の引数を処理 ---
        --height)
          HEIGHT="$2"
          shift 2
          ;;
        --width)
          WIDTH="$2"
          shift 2
          ;;
        --past-len)
          PAST_LEN="$2"
          shift 2
          ;;
        --odom-dim)
          ODOM_DIM="$2"
          shift 2
          ;;
        --image-tensor-name)
          IMAGE_TENSOR_NAME="$2"
          shift 2
          ;;
        --odom-tensor-name)
          ODOM_TENSOR_NAME="$2"
          shift 2
          ;;
        # ---
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

# (Check required arguments )
if [[ -z "$INPUT_ONNX_PATH" ]] || [[ -z "$MODEL_NAME" ]]; then
  echo "❌ Error: --input-onnx and --model-name are required arguments."
  show_help
  exit 1
fi

# Print parameters and run the setup
print_parameters
setup_model