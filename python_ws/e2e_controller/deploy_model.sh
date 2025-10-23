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

# This script first generates an ONNX model from a PyTorch checkpoint
# and then converts it to a TensorRT engine for deployment
# with Triton inside the Isaac ROS environment.

# === Default arguments ===
INPUT_ONNX_PATH="" # -i が指定されない場合、インタラクティブに生成
MODEL_NAME=""
HEIGHT="120"
WIDTH="160"
INPUT_TENSOR_NAME="input_1"
CONFIG_FILE=""
PRECISION="fp16"
MAX_BATCH_SIZE="1"
PYTHON_CONVERT_SCRIPT="3_convert_weight_format.py" # ONNX変換スクリプト名
CHECKPOINT_BASE_DIR="../ckpts/pilotnet"          # チェックポイントのベースディレクトリ

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
  local generated_onnx_name="model.onnx" # Tritonはmodel.onnxという名前を期待することがある

  # Copy the generated ONNX model to the destination directory
  echo "Copying ONNX model from ${INPUT_ONNX_PATH} to ${version_path}/${generated_onnx_name}"
  cp "${INPUT_ONNX_PATH}" "${version_path}/${generated_onnx_name}"

  echo "Converting ONNX model to a TensorRT Engine Plan (.plan)..."

  # Use trtexec to convert ONNX to a TensorRT engine
  /usr/src/tensorrt/bin/trtexec \
    --onnx="${version_path}/${generated_onnx_name}" \
    --saveEngine="${version_path}/model.plan" \
    --minShapes=${INPUT_TENSOR_NAME}:1x3x${HEIGHT}x${WIDTH} \
    --optShapes=${INPUT_TENSOR_NAME}:1x3x${HEIGHT}x${WIDTH} \
    --maxShapes=${INPUT_TENSOR_NAME}:${MAX_BATCH_SIZE}x3x${HEIGHT}x${WIDTH} \
    --${PRECISION} \
    --verbose
  
  # Copy the Triton configuration file
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
  echo "Usage: $0 [options]"
  echo "This script interactively generates an ONNX file from a checkpoint"
  echo "and deploys it as a TensorRT engine for Triton."
  echo
  echo "Options:"
  echo "  -i, --input-onnx        [OPTIONAL] Path to an *existing* ONNX model file."
  echo "                          If omitted, the script will interactively generate one."
  echo "  -m, --model-name        Name for the model. If omitted, an interactive menu will appear."
  echo "  -c, --config-file       Name of the pbtxt config file. If omitted, it will be inferred from --model-name."
  echo "  -p, --precision         Precision for TensorRT engine (fp32, fp16, int8). (Default: ${PRECISION})"
  echo "  -b, --max-batch-size    Maximum batch size for the TensorRT engine. (Default: ${MAX_BATCH_SIZE})"
  echo "      --height            Input image height. (Default: ${HEIGHT})"
  echo "      --width             Input image width. (Default: ${WIDTH})"
  echo "      --input-name        Name of the input tensor in the ONNX model. (Default: ${INPUT_TENSOR_NAME})"
  echo "  -h, --help              Show this help message."
}

# --- 新: インタラクティブなチェックポイント選択 ---
# (プロンプトをstderrに出力し、選択結果をstdoutに出力)
function select_checkpoint_interactive() {
  echo "--- チェックポイントのディレクトリを検索しています (${CHECKPOINT_BASE_DIR}) ---" >&2
  
  local options=()
  # findでディレクトリ(-type d)のみを検索し、ソートして配列に格納
  while IFS= read -r dir; do
    options+=("$(basename "$dir")")
  done < <(find "${CHECKPOINT_BASE_DIR}" -mindepth 1 -maxdepth 1 -type d | sort)

  if [[ ${#options[@]} -eq 0 ]]; then
    echo "❌ Error: ${CHECKPOINT_BASE_DIR} にチェックポイントのディレクトリが見つかりません。" >&2
    exit 1
  fi
  options+=("終了 (Quit)")

  echo "--- チェックポイントのディレクトリを選択してください ---" >&2
  PS3="番号を入力してください: "
  
  while true; do
    select opt in "${options[@]}"; do
      if [[ "$opt" == "終了 (Quit)" ]]; then
        echo "スクリプトを終了します。" >&2
        exit 1
      elif [[ -n "$opt" ]]; then
        local selected_dir_path="${CHECKPOINT_BASE_DIR}/${opt}"
        # ユーザーの指示に基づき、best_model.pth を探す
        # (元の指示 .sh は .pth のタイポと判断)
        local pth_file="${selected_dir_path}/best_model.pth"
        
        if [[ ! -f "$pth_file" ]]; then
           echo "❌ Error: $pth_file が見つかりません。 (ディレクトリ: $opt)" >&2
           break # selectループを抜け、メニューを再表示
        else
           echo "$pth_file" # 最終的なパスをstdoutに出力
           return 0
        fi
      else
        echo "無効な選択です。" >&2
        break
      fi
    done
  done
}

# --- 新: インタラクティブなモデルサイズ選択 ---
# (プロンプトをstderrに出力し、選択結果をstdoutに出力)
function select_model_size_interactive() {
  echo "--- モデルのサイズを選択してください ---" >&2
  PS3="番号を入力してください: "
  local options=("tiny" "normal" "終了 (Quit)")
  
  while true; do
    select opt in "${options[@]}"; do
      case $opt in
        "tiny" | "normal")
          echo "$opt" # 選択結果をstdoutに出力
          return 0
          ;;
        "終了 (Quit)")
          echo "スクリプトを終了します。" >&2
          exit 1
          ;;
        *) 
          echo "無効な選択です。" >&2
          break # selectループを抜け、メニューを再表示
          ;;
      esac
    done
  done
}


# --- 既存: インタラクティブなモデル名選択 ---
function select_model_interactive() {
  echo "--- デプロイ先のモデル名を選択してください ---"
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


# --- [拡張] ONNXファイルの決定ロジック ---
if [[ -z "$INPUT_ONNX_PATH" ]]; then
  echo "ℹ️ -i (--input-onnx) が指定されていません。インタラクティブにONNXを生成します。"
  
  # 1. チェックポイントディレクトリを選択
  # (select_checkpoint_interactive は結果をstdoutに出力する)
  SELECTED_CHECKPOINT_PATH=$(select_checkpoint_interactive)
  # 選択がキャンセルされたかチェック
  if [[ $? -ne 0 ]] || [[ -z "$SELECTED_CHECKPOINT_PATH" ]]; then
    echo "❌ チェックポイントの選択がキャンセルされました。"
    exit 1
  fi
  echo "✅ チェックポイント: $SELECTED_CHECKPOINT_PATH を選択しました。"

  # 2. モデルサイズを選択
  SELECTED_MODEL_SIZE=$(select_model_size_interactive)
  if [[ $? -ne 0 ]] || [[ -z "$SELECTED_MODEL_SIZE" ]]; then
    echo "❌ モデルサイズの選択がキャンセルされました。"
    exit 1
  fi
  echo "✅ サイズ: $SELECTED_MODEL_SIZE を選択しました。"

  # 3. Pythonスクリプトの存在確認
  if [[ ! -f "$PYTHON_CONVERT_SCRIPT" ]]; then
      echo "❌ Error: ONNX変換スクリプト '$PYTHON_CONVERT_SCRIPT' がカレントディレクトリに見つかりません。"
      exit 1
  fi
  
  # 4. Pythonスクリプトを実行してONNXを生成
  echo "🔄 実行中: python3 $PYTHON_CONVERT_SCRIPT --checkpoint $SELECTED_CHECKPOINT_PATH --size $SELECTED_MODEL_SIZE"
  python3 "$PYTHON_CONVERT_SCRIPT" --checkpoint "$SELECTED_CHECKPOINT_PATH" --size "$SELECTED_MODEL_SIZE"

  if [[ $? -ne 0 ]]; then
    echo "❌ Error: ONNXの生成に失敗しました。"
    exit 1
  fi

  # 5. 生成されたONNXファイルのパスを変数に設定
  # (.pth と同じディレクトリに best_model.onnx が生成されると仮定)
  
  # ★★★ 修正箇所 ★★★
  # 'local' を削除
  checkpoint_dir=$(dirname "$SELECTED_CHECKPOINT_PATH")
  # ★★★ 修正箇所 ★★★
  
  INPUT_ONNX_PATH="${checkpoint_dir}/best_model.onnx"

  if [[ ! -f "$INPUT_ONNX_PATH" ]]; then
    echo "❌ Error: ONNXファイルが期待されたパス ($INPUT_ONNX_PATH) に生成されませんでした。"
    exit 1
  fi
  
  echo "✅ ONNXファイルが正常に生成されました: $INPUT_ONNX_PATH"

else
  echo "✅ -i $INPUT_ONNX_PATH が指定されたため、ONNXの生成をスキップします。"
fi
# --- [拡張] ONNXロジックここまで ---


# --- [既存] MODEL_NAME / CONFIG_FILE の決定ロジック ---
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
      "pilotnet_parking")
        CONFIG_FILE="pilotnet_parking_config.pbtxt"
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


# Check if required arguments are provided (INPUT_ONNX_PATH はこの時点で設定されているはず)
if [[ -z "$INPUT_ONNX_PATH" ]] || [[ -z "$MODEL_NAME" ]] || [[ -z "$CONFIG_FILE" ]]; then
  echo "❌ Error: 必須項目が不足しています。"
  echo "  INPUT_ONNX_PATH (生成または指定): ${INPUT_ONNX_PATH:-<未設定>}"
  echo "  MODEL_NAME (選択または指定)     : ${MODEL_NAME:-<未設定>}"
  echo "  CONFIG_FILE (選択または指定)    : ${CONFIG_FILE:-<未設定>}"
  show_help
  exit 1
fi

# Print parameters and run the setup
print_parameters
setup_model