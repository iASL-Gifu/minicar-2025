#!/bin/bash

# --- 設定項目 ---
DEFAULT_SESSION_NAME="tamiya"                     # デフォルトのセッション名
WINDOW_NAME="main"                             # ウィンドウ名
ROS_WS_PATH="$HOME/workspace/minicar_2025/ros2_ws" # ROS 2ワークスペースのパス 
SETUP_SCRIPT="source install/setup.bash"

# --- セッション名の決定 ---
if [ -n "$1" ]; then
  SESSION_NAME="$1"
else
  SESSION_NAME="$DEFAULT_SESSION_NAME"
fi

# --- tmuxセッションの準備 ---
# セッションが既に存在するか確認
tmux has-session -t $SESSION_NAME 2>/dev/null

if [ $? != 0 ]; then
  # セッションが存在しない場合、新しいセッションとペインを作成・設定する

  # 1. 新しいセッションを作成
  tmux new-session -d -s $SESSION_NAME -n $WINDOW_NAME

  # 2. 最初のペイン(0)を縦に分割 -> 上(0)と下(1)ができる
  tmux split-window -v -t $SESSION_NAME:$WINDOW_NAME.0

  # 3. 上のペイン(0)を横に分割 -> 左上(0)と右上(2)ができる
  tmux split-window -h -t $SESSION_NAME:$WINDOW_NAME.0

  # 4. 下のペイン(1)を横に分割 -> 左下(1)と右下(3)ができる
  tmux split-window -h -t $SESSION_NAME:$WINDOW_NAME.1

  # --- 各ペインでコマンドを実行 ---
  # 左上 (ペイン0)
  tmux send-keys -t $SESSION_NAME:$WINDOW_NAME.0 "cd $ROS_WS_PATH && $SETUP_SCRIPT && clear" C-m
  
  # 左下 (ペイン1)
  tmux send-keys -t $SESSION_NAME:$WINDOW_NAME.1 "cd $ROS_WS_PATH && $SETUP_SCRIPT && clear" C-m

  # 右上 (ペイン2)
  tmux send-keys -t $SESSION_NAME:$WINDOW_NAME.2 "cd $ROS_WS_PATH && $SETUP_SCRIPT && clear" C-m

  # 右下 (ペイン3)
  tmux send-keys -t $SESSION_NAME:$WINDOW_NAME.3 "cd $ROS_WS_PATH && $SETUP_SCRIPT && clear" C-m

  # 最後にアクティブにするペインを選択 (例: 左上のペイン0)
  tmux select-pane -t $SESSION_NAME:$WINDOW_NAME.0

fi

# セッションにアタッチ
tmux attach-session -t $SESSION_NAME