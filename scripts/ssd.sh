#!/bin/bash

# --- 設定項目 (環境に合わせて変更してください) ---
# コピー元の基準ディレクトリ
SOURCE_BASE_DIR="/home/tamiya/workspace/minicar-2025/ros2_ws/record"
# マウントするデバイス名
DEVICE="/dev/sda1"
# マウントポイント
MOUNT_POINT="/mnt/ssd"
# SSD内の保存先サブディレクトリ名
DEST_SUBDIR="rosbag-minicar"
# ----------------------------------------------------

# --- 0. sudo 権限の事前確認 ---
echo "ℹ️ このスクリプトは、ディスクのマウントと書き込みのために root 権限（sudo）を必要とします。"
sudo -v
if [ $? -ne 0 ]; then
  echo "❌ エラー: sudo 権限の取得に失敗しました。"
  exit 1
fi
echo "✅ sudo 権限を確認しました。"
echo "----------------------------------------"

# --- 1. 物理的な接続の確認 ---
read -p "❓ SSDドライブをPCに接続しましたか？ (y/n): " confirm
if [[ "$confirm" != [yY] ]]; then
  echo "処理を中断しました。"
  exit 0
fi
echo "----------------------------------------"

# --- 2. コピー対象の選択 ---
echo "⏳ コピー元のディレクトリ一覧を取得中..."
mapfile -t DIRS < <(find "$SOURCE_BASE_DIR" -mindepth 1 -maxdepth 1 -type d -printf '%f\n')

if [ ${#DIRS[@]} -eq 0 ]; then
  echo "❌ エラー: $SOURCE_BASE_DIR 内にコピー対象のディレクトリが見つかりませんでした。"
  exit 1
fi

echo "✅ ディレクトリ一覧の取得完了。"
echo "以下のディレクトリからコピー対象を選択してください："
echo ""

i=1
for d in "${DIRS[@]}"; do
  echo "  [$i] $d"
  ((i++))
done
echo ""

read -p "番号 (例: 1 3 5 / 2-4 / all / q): " selections

if [[ "$selections" =~ ^(q|quit)$ ]]; then
  echo "処理を終了します。"
  exit 0
fi

TARGET_DIRS=()

if [[ "$selections" == "all" ]]; then
  for d in "${DIRS[@]}"; do
    TARGET_DIRS+=("${SOURCE_BASE_DIR}/${d}")
  done
else
  for token in $selections; do
    if [[ "$token" =~ ^[0-9]+-[0-9]+$ ]]; then
      start=${token%-*}
      end=${token#*-}
      for ((idx=start; idx<=end; idx++)); do
        TARGET_DIRS+=("${SOURCE_BASE_DIR}/${DIRS[idx-1]}")
      done
    elif [[ "$token" =~ ^[0-9]+$ ]]; then
      TARGET_DIRS+=("${SOURCE_BASE_DIR}/${DIRS[token-1]}")
    fi
  done
fi

echo "----------------------------------------"

# --- 3. 事前チェック ---
echo "✅ 事前チェックを開始します..."
if [ ${#TARGET_DIRS[@]} -eq 0 ]; then
  echo "❌ エラー: コピー対象のディレクトリが選択されませんでした。"
  exit 1
fi

echo " - コピー対象一覧:"
for dir in "${TARGET_DIRS[@]}"; do
  echo "   - $dir"
done
echo "----------------------------------------"

# --- 4. マウント処理 ---
if ! findmnt -M "$MOUNT_POINT" > /dev/null; then
  echo "🔄 $DEVICE はマウントされていません。マウントします..."
  sudo mkdir -p "$MOUNT_POINT"
  sudo mount "$DEVICE" "$MOUNT_POINT"
else
  echo "✅ $MOUNT_POINT は既にマウントされています。"
fi
echo "----------------------------------------"

# --- 5. データ転送処理 ---
DEST_DIR="${MOUNT_POINT}/${DEST_SUBDIR}"
sudo mkdir -p "$DEST_DIR"
echo "🔄 データの転送を開始します..."
echo " - To: $DEST_DIR"

for source_path in "${TARGET_DIRS[@]}"; do
  echo ""
  echo "▶️ コピー中: $(basename "$source_path")"
  sudo rsync -avh --progress "$source_path" "$DEST_DIR"
done

echo ""
echo "✅ 全てのデータ転送が完了しました。"
echo "----------------------------------------"

# --- 6. アンマウント処理 ---
echo "🔄 アンマウント処理を開始します..."
sudo sync
sudo umount "$MOUNT_POINT"
echo "✅ アンマウントに成功しました。"
echo "----------------------------------------"
echo "🎉 すべての処理が正常に完了しました。"
exit 0
