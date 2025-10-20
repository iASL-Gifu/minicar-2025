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
MODEL_NAME=""      # デフォルトを空にし、後でインタラクティブに設定
HEIGHT="120"
WIDTH="160"
INPUT_TENSOR_NAME="input_1"
CONFIG_FILE=""     # デフォルトを空にし、後でインタラクティブに設定
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
  local output_path="/workspaces/isaac_ros_assets/models/${MODEL_NAME}"
  
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
  local config_source_path="${pkg_share_path}/config/${CONFIG_FILE}"

  # もし見つからなければ、カレントディレクトリも探す
  if [[ ! -f "$config_source_path" ]]; then
    if [[ -f "./${CONFIG_FILE}" ]]; then
      config_source_path="./${CONFIG_FILE}"
      echo "ℹ️ ROS 2 package path not found, using config file from current directory."
    else
      echo "❌ Error: Config file '${CONFIG_FILE}' not found in ROS 2 package or current directory."
      exit 1
    fi
  fi
  
  echo "Using config file from: ${config_source_path}"
  cp "${config_source_path}" "${output_path}/config.pbtxt"
    
  echo "✅ Completed model setup for ${MODEL_NAME} (version ${version})."
}

function show_help() {
  echo "Usage: $0 -i /path/to/model.onnx [options]"
  echo "Options:"
  echo "  -i, --input-onnx        [REQUIRED] Path to the local ONNX model file."
  echo "  -m, --model-name        Name for the model. If omitted, an interactive menu will appear."
  echo "  -c, --config-file       Name of the pbtxt config file. If omitted, it will be inferred from --model-name."
  echo "  -p, --precision         Precision for TensorRT engine (fp32, fp16, int8). (Default: ${PRECISION})"
  echo "  -b, --max-batch-size    Maximum batch size for the TensorRT engine. (Default: ${MAX_BATCH_SIZE})"
  echo "      --height            Input image height. (Default: ${HEIGHT})"
  echo "      --width             Input image width. (Default: ${WIDTH})"
  echo "      --input-name        Name of the input tensor in the ONNX model. (Default: ${INPUT_TENSOR_NAME})"
  echo "  -h, --help              Show this help message."
}

# --- 新しい関数: インタラクティブなモデル選択 ---
function select_model_interactive() {
  echo "--- モデルを選択してください ---"
  PS3="番号を入力してください: "
  options=("pilotnet" "pilotnet_540" "pilotnet_race" "手動入力 (Manual Input)" "終了 (Quit)")
  
  while true; do
    select opt in "${options[@]}"; do
      case $opt in
        "pilotnet")
          MODEL_NAME="pilotnet"
          CONFIG_FILE="pilotnet_config.pbtxt"
          echo "✅ モデル: $MODEL_NAME, 設定: $CONFIG_FILE を選択しました。"
          break 2 # whileループを抜ける
          ;;
        "pilotnet_540")
          MODEL_NAME="pilotnet_540"
          CONFIG_FILE="pilotnet_540_config.pbtxt"
          echo "✅ モデル: $MODEL_NAME, 設定: $CONFIG_FILE を選択しました。"
          break 2
          ;;
        "pilotnet_race")
          MODEL_NAME="pilotnet_race"
          CONFIG_FILE="pilotnet_race_config.pbtxt"
          echo "✅ モデル: $MODEL_NAME, 設定: $CONFIG_FILE を選択しました。"
          break 2
          ;;
        "手動入力 (Manual Input)")
          while [[ -z "$MODEL_NAME" ]]; do
            read -p "モデル名を入力してください: " MODEL_NAME
          done
          while [[ -z "$CONFIG_FILE" ]]; do
            read -p "設定ファイル名 (e.g., my_config.pbtxt) を入力してください: " CONFIG_FILE
          done
          echo "✅ モデル: $MODEL_NAME, 設定: $CONFIG_FILE を入力しました。"
          break 2
          ;;
        "終了 (Quit)")
          echo "スクリプトを終了します。"
          exit 0
          ;;
        *) 
          echo "無効な選択です。1-${#options[@]} の番号を選んでください。"
          break # selectループだけ抜ける (whileは継続)
          ;;
      esac
    done
  done
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


if [[ -z "$MODEL_NAME" ]]; then
  # -m が指定されなかった場合、インタラクティブメニューを起動
  echo "ℹ️ -m (model-name) が指定されていません。インタラクティブ・メニューを開始します。"
  select_model_interactive
else
  # -m が指定された場合
  echo "✅ -m $MODEL_NAME が指定されました。"
  if [[ -z "$CONFIG_FILE" ]]; then
    # -c が指定されていない場合、MODEL_NAMEから自動マッピング
    echo "ℹ️ -c (config-file) が未指定のため、モデル名から推測します..."
    case "$MODEL_NAME" in
      "pilotnet")
        CONFIG_FILE="pilotnet_config.pbtxt"
        ;;
      "pilotnet_540")
        CONFIG_FILE="pilotnet_540_config.pbtxt"
        ;;
      "pilotnet_race")
        CONFIG_FILE="pilotnet_race_config.pbtxt"
        ;;
      *)
        echo "⚠️ 既知のモデル名と一致しません。configファイルを手動で入力してください。"
        while [[ -z "$CONFIG_FILE" ]]; do
          read -p "設定ファイル名 (e.g., ${MODEL_NAME}_config.pbtxt): " CONFIG_FILE
        done
        ;;
    esac
    echo "✅ 設定ファイル: $CONFIG_FILE を使用します。"
  else
    # -m と -c の両方が指定された場合
    echo "✅ -c $CONFIG_FILE が指定されました。"
  fi
fi


# Check if required arguments are provided
if [[ -z "$INPUT_ONNX_PATH" ]] || [[ -z "$MODEL_NAME" ]] || [[ -z "$CONFIG_FILE" ]]; then
  echo "❌ Error: 必須項目が不足しています。"
  echo "  --input-onnx は必須です。"
  echo "  --model-name (またはインタラクティブ選択) が必要です。"
  echo "  --config-file (または自動マッピング) が必要です。"
  show_help
  exit 1
fi

# Print parameters and run the setup
print_parameters
setup_model