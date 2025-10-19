#!/bin/bash

# --- 設定項目 (環境に合わせて変更してください) ---

# コピー元の基準ディレクトリ
SOURCE_BASE_DIR="/home/tamiya/workspaces/minicar-2025/ros2_ws/record"

# マウントするデバイス名
DEVICE="/dev/sda1"

# マウントポイント
MOUNT_POINT="/mnt/ssd"

# SSD内の保存先サブディレクトリ名
DEST_SUBDIR="rosbag-minicar"
# ----------------------------------------------------

# スクリプトがroot権限で実行されていない場合、sudoを使って自身を再実行する
if [ "$EUID" -ne 0 ]; then
  echo "INFO: root権限が必要です。sudoを使って再実行します..."
  exec sudo "$0" "$@"
fi

# --- 1. 物理的な接続の確認 ---
read -p "❓ SSDドライブをPCに接続しましたか？ (y/n): " confirm
if [[ "$confirm" != [yY] ]]; then
    echo "処理を中断しました。"
    exit 0
fi
echo "----------------------------------------"

# --- 2. コピー対象の選択 ---
echo "⏳ コピー元のディレクトリ一覧を取得中..."
# SOURCE_BASE_DIR 直下のディレクトリを検索して DIRS 配列に格納
mapfile -t DIRS < <(find "$SOURCE_BASE_DIR" -mindepth 1 -maxdepth 1 -type d -printf '%f\n')

if [ ${#DIRS[@]} -eq 0 ]; then
    echo "❌ エラー: $SOURCE_BASE_DIR 内にコピー対象のディレクトリが見つかりませんでした。"
    exit 1
fi

echo "✅ ディレクトリ一覧の取得完了。"
echo "どのディレクトリをSSDにコピーしますか？"

# select構文で選択肢を表示
PS3="番号を入力してください (qで終了): "
options=("全てコピー" "${DIRS[@]}" "quit")
select choice in "${options[@]}"; do
    case "$choice" in
        "quit")
            echo "処理を終了します。"
            exit 0
            ;;
        "全てコピー")
            SOURCE_DIR="$SOURCE_BASE_DIR"
            echo "✅ 全てのディレクトリをコピー対象とします。"
            break
            ;;
        "")
            echo "無効な選択です。リストから番号を選んでください。"
            ;;
        *)
            SOURCE_DIR="${SOURCE_BASE_DIR}/${choice}"
            echo "✅ 「${choice}」をコピー対象とします。"
            break
            ;;
    esac
done
echo "----------------------------------------"

# --- 3. 事前チェック ---
echo "✅ 事前チェックを開始します..."

if [ ! -d "$SOURCE_DIR" ]; then
    echo "❌ エラー: コピー元ディレクトリ $SOURCE_DIR が存在しません。"
    exit 1
fi

echo "  - コピー元: $SOURCE_DIR"
echo "  - デバイス: $DEVICE"
echo "  - マウント先: $MOUNT_POINT"
echo "----------------------------------------"


# --- 4. マウント処理 ---
if ! findmnt -M "$MOUNT_POINT" > /dev/null; then
    echo "🔄 $DEVICE はマウントされていません。マウントします..."
    mkdir -p "$MOUNT_POINT"
    if [ $? -ne 0 ]; then
        echo "❌ エラー: マウントポイント $MOUNT_POINT の作成に失敗しました。"
        exit 1
    fi
    mount "$DEVICE" "$MOUNT_POINT"
    if [ $? -ne 0 ]; then
        echo "❌ エラー: マウントに失敗しました。デバイス名が正しいか確認してください。"
        exit 1
    fi
    echo "✅ マウントに成功しました。"
else
    echo "✅ $MOUNT_POINT は既にマウントされています。"
fi
echo "----------------------------------------"


# --- 5. データ転送処理 ---
DEST_DIR="${MOUNT_POINT}/${DEST_SUBDIR}"

echo "🔄 データの転送を開始します..."
echo "  - From: $SOURCE_DIR"
echo "  - To:   $DEST_DIR"

mkdir -p "$DEST_DIR"

# rsync を実行。コピー元ディレクトリの末尾に / を付けることで、ディレクトリの中身だけをコピーする
rsync -avh --progress "$SOURCE_DIR/" "$DEST_DIR"

if [ $? -ne 0 ]; then
    echo "❌ エラー: データ転送に失敗しました。"
    echo "ディスクの空き容量や権限を確認してください。"
    echo "安全のため、アンマウントせずにスクリプトを終了します。"
    exit 1
fi

echo "✅ データ転送が完了しました。"
echo "----------------------------------------"


# --- 6. アンマウント処理 ---
echo "🔄 アンマウント処理を開始します..."
echo "  - データを同期中 (sync)..."
sync
umount "$MOUNT_POINT"

if [ $? -ne 0 ]; then
    echo "❌ エラー: アンマウントに失敗しました。何らかのプロセスがビジー状態かもしれません。"
    exit 1
fi

echo "✅ アンマウントに成功しました。"
echo "----------------------------------------"
echo "🎉 すべての処理が正常に完了しました。"

exit 0

