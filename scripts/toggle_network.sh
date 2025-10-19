#!/bin/bash

# スクリプト名
SCRIPT_NAME=$(basename "$0")

# ヘルプメッセージを表示する関数
show_help() {
  echo "使用法: sudo $SCRIPT_NAME [インターフェース名]"
  echo "ネットワークインターフェースの状態を UP/DOWN で切り替えます。"
  echo
  echo "引数:"
  echo "  インターフェース名   対象のインターフェース名 (例: enp3s0, wlan0)"
  echo "                     省略した場合はデフォルト値 'wlP1p1s0' が使用されます。"
  echo
  echo "利用可能なインターフェース:"
  ip -br link
}

# 引数が --help または -h の場合はヘルプを表示して終了
if [[ "$1" == "--help" || "$1" == "-h" ]]; then
  show_help
  exit 0
fi

# 1. インターフェース名を設定
# コマンドライン引数があればその値を、なければデフォルト値 "wlP1p1s0" を使用
INTERFACE=${1:-"wlP1p1s0"}

# 2. インターフェースの存在と現在の状態を確認
# -o オプションで1行表示にし、awkで9番目のフィールド（状態）を取得
CURRENT_STATE=$(ip -o link show "$INTERFACE" 2>/dev/null | awk '{print $9}')

# インターフェースが存在しない場合のエラー処理
if [ -z "$CURRENT_STATE" ]; then
  echo "エラー: インターフェース '$INTERFACE' が見つかりません。"
  echo "利用可能なインターフェースを確認してください:"
  echo "--------------------"
  ip -br link
  echo "--------------------"
  exit 1
fi

echo "✅ 対象インターフェース: $INTERFACE (現在の状態: $CURRENT_STATE)"

# 3. 状態に応じて処理を分岐
if [ "$CURRENT_STATE" == "UP" ]; then
  # UP の場合は DOWN にする
  echo "🔌 ---> '$INTERFACE' を DOWN にします..."
  sudo ip link set "$INTERFACE" down
else
  # DOWN またはその他の状態の場合は UP にする
  echo "⚡️ ---> '$INTERFACE' を UP にします..."
  sudo ip link set "$INTERFACE" up
fi

# 4. 実行後の状態を確認して表示
NEW_STATE=$(ip -o link show "$INTERFACE" | awk '{print $9}')
echo "👍 完了しました。新しい状態: $NEW_STATE"