#!/bin/bash

# --- 設定 ---
DEVICE="/dev/sda1"
MOUNT_POINT="/mnt/ssd"
# ------------

# スクリプトをroot権限 (sudo) で実行しているかチェック
if [ "$EUID" -ne 0 ]; then
  echo "このスクリプトは sudo を使って実行してください (例: sudo $0)"
  exit 1
fi

# マウントポイントのディレクトリが存在するか確認
if [ ! -d "$MOUNT_POINT" ]; then
    echo "マウントポイント $MOUNT_POINT が存在しません。"
    echo "作成します..."
    mkdir -p "$MOUNT_POINT"
    if [ $? -ne 0 ]; then
        echo "マウントポイントの作成に失敗しました。"
        exit 1
    fi
    echo "マウントポイントを作成しました。"
fi

# findmnt コマンドで、指定したマウントポイントが現在マウントされているか確認
if findmnt -M "$MOUNT_POINT" > /dev/null; then
    # 1. マウントされている場合 (アンマウント処理)
    echo "$MOUNT_POINT はマウントされています。アンマウントします..."
    
    # データをディスクに同期 (sync)
    echo "データを同期中 (sync)..."
    sync
    
    # アンマウント実行
    umount "$MOUNT_POINT"
    
    if [ $? -eq 0 ]; then
        echo "アンマウントに成功しました。"
    else
        echo "エラー: アンマウントに失敗しました。ビジー状態かもしれません。"
        exit 1
    fi
    
else
    # 2. マウントされていない場合 (マウント処理)
    echo "$DEVICE はマウントされていません。マウントします..."
    
    # マウント実行
    mount "$DEVICE" "$MOUNT_POINT"
    
    if [ $? -eq 0 ]; then
        echo "マウントに成功しました。"
    else
        echo "エラー: マウントに失敗しました。デバイス名が正しいか確認してください。"
        exit 1
    fi
fi

exit 0