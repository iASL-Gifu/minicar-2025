#!/bin/bash

# tmux セッション名
SESSION_NAME="work"

# tmuxサーバーを起動（既に起動していても問題なし）
tmux start-server

# 既存のセッションがあれば削除
tmux kill-session -t $SESSION_NAME 2>/dev/null

# 新しいセッションを作成
tmux new-session -d -s $SESSION_NAME

# ペインを4分割にする（横2x縦2）
# ペイン0（左上）
tmux send-keys -t $SESSION_NAME "clear" Enter
tmux send-keys -t $SESSION_NAME "/scripts/global_localization/2.vslam.sh"

# ペイン1（右上）を作成
tmux split-window -h -t $SESSION_NAME
tmux send-keys -t $SESSION_NAME "clear" Enter
tmux send-keys -t $SESSION_NAME "/scripts/global_localization/2.split_data.sh"

# ペイン2（左下）を作成
tmux split-window -v -t $SESSION_NAME:0.0
tmux send-keys -t $SESSION_NAME "clear" Enter
tmux send-keys -t $SESSION_NAME "/scripts/global_localization/2.play_data.sh"

# ペイン3（右下）を作成
tmux split-window -v -t $SESSION_NAME:0.1
tmux send-keys -t $SESSION_NAME "clear" Enter
tmux send-keys -t $SESSION_NAME "/scripts/global_localization/2.record_data.sh"

# ペインのレイアウトを均等にする
tmux select-layout -t $SESSION_NAME tiled

# tmuxセッションにアタッチ
tmux attach-session -t $SESSION_NAME