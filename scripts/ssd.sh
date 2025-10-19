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
echo "パスワードの入力を求められる場合があります。"
sudo -v # sudo の認証タイムスタンプを更新する（必要ならパスワードを尋ねる）
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
# SOURCE_BASE_DIR 直下のディレクトリを検索して DIRS 配列に格納
mapfile -t DIRS < <(find "$SOURCE_BASE_DIR" -mindepth 1 -maxdepth 1 -type d -printf '%f\n')

if [ ${#DIRS[@]} -eq 0 ]; then
    echo "❌ エラー: $SOURCE_BASE_DIR 内にコピー対象のディレクトリが見つかりませんでした。"
    exit 1
fi

echo "✅ ディレクトリ一覧の取得完了。"
echo "どのディレクトリをSSDにコピーしますか？"

# ★ コピー対象のディレクトリパスを格納する配列
TARGET_DIRS=()

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
            echo "✅ 全てのディレクトリをコピー対象とします。"
            # ★ SOURCE_BASE_DIR 直下の全ディレクトリをコピー対象に追加
            for dir in "${DIRS[@]}"; do
                TARGET_DIRS+=("${SOURCE_BASE_DIR}/${dir}")
            done
            break
            ;;
        "")
            echo "無効な選択です。リストから番号を選んでください。"
            ;;
        *)
            echo "✅ 「${choice}」をコピー対象とします。"
            # ★ 選択されたディレクトリをコピー対象に追加
            TARGET_DIRS+=("${SOURCE_BASE_DIR}/${choice}")
            break
            ;;
    esac
done
echo "----------------------------------------"

# --- 3. 事前チェック ---
echo "✅ 事前チェックを開始します..."

if [ ${#TARGET_DIRS[@]} -eq 0 ]; then
    echo "❌ エラー: コピー対象のディレクトリが選択されませんでした。"
    exit 1
fi

# 選択されたコピー対象を一覧表示
echo "  - コピー対象:"
for dir in "${TARGET_DIRS[@]}"; do
    if [ ! -d "$dir" ]; then
        echo "❌ エラー: コピー元ディレクトリ $dir が存在しません。"
        exit 1
    fi
    echo "    - $dir"
done

echo "  - デバイス: $DEVICE"
echo "  - マウント先: $MOUNT_POINT"
echo "----------------------------------------"


# --- 4. マウント処理 ---
if ! findmnt -M "$MOUNT_POINT" > /dev/null; then
    echo "🔄 $DEVICE はマウントされていません。マウントします..."
    sudo mkdir -p "$MOUNT_POINT"
    if [ $? -ne 0 ]; then
        echo "❌ エラー: マウントポイント $MOUNT_POINT の作成に失敗しました。"
        exit 1
    fi
    sudo mount "$DEVICE" "$MOUNT_POINT"
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
sudo mkdir -p "$DEST_DIR"

echo "🔄 データの転送を開始します..."
echo "  - To:   $DEST_DIR"

# ★ 選択された各ディレクトリをループでコピー
for source_path in "${TARGET_DIRS[@]}"; do
    echo ""
    echo "▶️  コピー中: $(basename "$source_path")"
    # ★★★ rsync のソースパス末尾の "/" を削除 ★★★
    # これにより、ディレクトリ自体がコピー先に作成される
    sudo rsync -avh --progress "$source_path" "$DEST_DIR"
    
    if [ $? -ne 0 ]; then
        echo "❌ エラー: 「$(basename "$source_path")」のデータ転送に失敗しました。"
        echo "ディスクの空き容量や権限を確認してください。"
        echo "安全のため、アンマウントせずにスクリプトを終了します。"
        exit 1
    fi
done

echo ""
echo "✅ 全てのデータ転送が完了しました。"
echo "----------------------------------------"


# --- 6. アンマウント処理 ---
echo "🔄 アンマウント処理を開始します..."
echo "  - データを同期中 (sync)..."
sudo sync
sudo umount "$MOUNT_POINT"

if [ $? -ne 0 ]; then
    echo "❌ エラー: アンマウントに失敗しました。何らかのプロセスがビジー状態かもしれません。"
    exit 1
fi

echo "✅ アンマウントに成功しました。"
echo "----------------------------------------"
echo "🎉 すべての処理が正常に完了しました。"

exit 0
