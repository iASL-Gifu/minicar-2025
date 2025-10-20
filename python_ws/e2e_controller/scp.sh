#!/bin/bash

# --- 設定 ---
REMOTE_USER="tamiya"
REMOTE_IP="10.42.0.1"
REMOTE_TARGET_DIR="/home/tamiya/workspace/minicar-2025/python_ws/ckpts/pilotnet"

# スクリプトが検索するローカルのチェックポイント・ベースディレクトリ
# （2_train.py の出力先に応じて変更してください）
LOCAL_CKPT_BASE_DIR="./ckpts/train"

# --- 色付け (オプション) ---
CYAN='\033[0;36m'
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[0;33m'
NC='\033[0m' # No Color

# --- 1. 最新のckptファイルを見つける ---
echo -e "🔄 Searching for the latest checkpoint file in: ${CYAN}$LOCAL_CKPT_BASE_DIR${NC}"

# .ckpt または .pth ファイルを再帰的に検索し、最終更新日時でソートして最新の1件を取得
LATEST_CKPT=$(find "$LOCAL_CKPT_BASE_DIR" -type f \( -name "*.ckpt" -o -name "*.pth" \) -print0 | xargs -0 ls -t | head -n 1)

if [ -z "$LATEST_CKPT" ]; then
    echo -e "${RED}ERROR: No .ckpt or .pth files found in $LOCAL_CKPT_BASE_DIR${NC}"
    echo -e "${YELLOW}Please check the LOCAL_CKPT_BASE_DIR variable in this script.${NC}"
    exit 1
fi

echo -e "Found latest file: ${GREEN}$(basename "$LATEST_CKPT")${NC}"
echo -e "Full path: $LATEST_CKPT"

# --- 2. 転送 ---
REMOTE_DESTINATION="${REMOTE_USER}@${REMOTE_IP}:${REMOTE_TARGET_DIR}"
echo -e "\n🚀 Transferring to ${CYAN}$REMOTE_DESTINATION${NC}..."

scp "$LATEST_CKPT" "$REMOTE_DESTINATION"

# --- 3. 結果確認 ---
if [ $? -eq 0 ]; then
    echo -e "\n${GREEN}✅ File transferred successfully.${NC}"
else
    echo -e "\n${RED}❌ ERROR: File transfer failed.${NC}"
    exit 1
fi