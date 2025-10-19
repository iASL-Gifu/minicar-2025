#!/bin/bash

# --- 設定項目 ---
DEFAULT_SESSION_NAME="tamiya"                       # デフォルトのセッション名
WINDOW_NAME="main"                                  # ウィンドウ名
ROS_WS_PATH="${ISAAC_ROS_WS}"                       # ROS 2ワークスペースのパス（必要なら利用）
SETUP_SCRIPT="source /workspaces/install/setup.bash" # setup.bashへのフルパスを指定

# --- 実行するコマンド群 ---
CMD_BASE="ros2 launch system_launch base_system.launch.xml"
CMD_E2E="ros2 launch e2e_launch"
CMD_LOCALIZATION="ros2 launch localization_launch"
CMD_BAG="ros2 launch bag_manager_py bag_manager_node.launch.xml"

# --- セッション名の決定 ---
if [ -n "$1" ]; then
  SESSION_NAME="$1"
else
  SESSION_NAME="$DEFAULT_SESSION_NAME"
fi

# --- tmuxセッションの準備 ---
# セッションが存在するかチェック（exit code 0 -> 存在する）
tmux has-session -t "$SESSION_NAME" 2>/dev/null
if [ $? -ne 0 ]; then
  # セッションが無ければ新規作成して4分割
  tmux new-session -d -s "$SESSION_NAME" -n "$WINDOW_NAME"

  # 1. 縦分割（上: pane 0, 下: pane 1）
  tmux split-window -v -t "$SESSION_NAME":"$WINDOW_NAME".0

  # 2. 上ペインを横分割（左上: pane 0, 右上: pane 2）
  tmux split-window -h -t "$SESSION_NAME":"$WINDOW_NAME".0

  # 3. 下ペインを横分割（左下: pane 1, 右下: pane 3）
  tmux split-window -h -t "$SESSION_NAME":"$WINDOW_NAME".1

  # --- 各ペインで初期化コマンドを実行（環境変数設定・setup読み込み・クリア） ---
  for pane in 0 1 2 3; do
    tmux send-keys -t "$SESSION_NAME":"$WINDOW_NAME".$pane \
      "export ROS_LOCALHOST_ONLY=0 && $SETUP_SCRIPT && clear" C-m
  done

  # --- 各ペインへ個別コマンドを送信 ---
  tmux send-keys -t "$SESSION_NAME":"$WINDOW_NAME".0 "$CMD_BASE" C-m          # 左上
  tmux send-keys -t "$SESSION_NAME":"$WINDOW_NAME".2 "$CMD_E2E" C-m           # 右上
  tmux send-keys -t "$SESSION_NAME":"$WINDOW_NAME".1 "$CMD_LOCALIZATION" C-m  # 左下
  tmux send-keys -t "$SESSION_NAME":"$WINDOW_NAME".3 "$CMD_BAG" C-m           # 右下

  # 最終的に左上ペインをアクティブにしておく
  tmux select-pane -t "$SESSION_NAME":"$WINDOW_NAME".0
fi

# --- セッションへアタッチ（既に存在していても接続） ---
tmux attach-session -t "$SESSION_NAME"
