#!/bin/bash

# セッション名を指定
SESSION_NAME="screen"

# 1. 既にscreenセッション内にいるか（$STY環境変数が設定されているか）確認
if [ -n "$STY" ]; then
    # 1a. セッション内にいる場合
    echo "既にscreenセッション（$STY）内にいます。デタッチします。"
    # "-d" は現在のセッションをデタッチする
    screen -d

else
    # 1b. セッション外（メインのターミナル）にいる場合
    # 2. "screen"という名前のセッションが（デタッチ状態で）存在するか確認
    if screen -ls | grep -q "\.${SESSION_NAME}\b"; then
        # 2a. 存在する場合
        echo "セッション '${SESSION_NAME}' にアタッチします。"
        screen -r ${SESSION_NAME}
    else
        # 2b. 存在しない場合
        echo "セッション '${SESSION_NAME}' を新規作成します。"
        screen -S ${SESSION_NAME}
    fi
fi