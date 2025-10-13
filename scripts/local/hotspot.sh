#!/bin/bash

# --- 引数のチェック ---
if [ "$#" -ne 3 ]; then
    echo "使い方: $0 <インターフェース名> <SSID> <パスワード>"
    echo "例: $0 wlan0 my_hotspot my_password"
    exit 1
fi

IFNAME="$1"
SSID="$2"
PASSWORD="$3"

# --- コマンドの実行 ---
echo "インターフェース '${IFNAME}' でホットスポットを作成します..."
echo "SSID: ${SSID}"
sudo nmcli dev wifi hotspot ifname "${IFNAME}" ssid "${SSID}" password "${PASSWORD}"

echo "ホットスポットが作成されました。"
