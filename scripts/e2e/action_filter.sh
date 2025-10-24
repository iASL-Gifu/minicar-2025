#!/bin/bash

# ==========================================================
# CUI Live Parameter Tuner for SectionalAckermannFilterNode
# 使い方:
# 1. ターミナル1で `ros2 launch ...` を実行し、ノードを起動する
# 2. ターミナル2でこのスクリプトを実行する (`./live_tune_filter.sh`)
# ==========================================================

# --- 設定 ---
# ★ C++コードのノード名に合わせてください (例: /sectional_ackermann_filter_node)
NODE_NAME="/control_filter_node" 
TOTAL_SECTIONS=10

# --- dialogコマンドの存在チェック ---
if ! command -v dialog &> /dev/null; then
    echo "エラー: 'dialog' がインストールされていません。"
    echo "sudo apt install dialog を実行してください。"
    exit 1
fi

# --- ノードの存在チェック ---
if ! ros2 node list | grep -q "$NODE_NAME"; then
    dialog --title "Error" --msgbox "ノード '$NODE_NAME' が見つかりません。\n\nLaunchファイルでノードを起動してから、このスクリプトを実行してください。" 10 60
    clear
    exit 1
fi

# --- メインループ ---
while true; do
    # 1. セクション選択メニュー
    menu_items=()
    for i in $(seq 1 $TOTAL_SECTIONS); do
        menu_items+=($i "Tune Section $i")
    done
    
    sec_num=$(dialog --backtitle "Ackermann Filter Live Tuner (Node: $NODE_NAME)" \
                   --title "Select Section to Tune" \
                   --menu "\nSelect a section or Exit:" 18 50 10 \
                   "${menu_items[@]}" \
                   "Exit" "Exit Tuner" \
                   2>&1 >/dev/tty)

    # Cancel (ESC) または "Exit" を選択
    if [ $? -ne 0 ] || [ "$sec_num" == "Exit" ]; then
        break
    fi

    # 2. 現在のパラメータを取得 (ros2 param get)
    #    (ノードから現在の値を取得し、ダイアログの初期値にします)
    prefix="sections.$sec_num"
    
    # エラー(stderr)を捨て、grepで数値のみ抽出し、失敗したらデフォルト(0.0)
    current_offset=$(ros2 param get $NODE_NAME $prefix.steer_offset 2>/dev/null | grep -oE '[-+]?[0-9]*\.?[0-9]+' || echo "0.0")
    current_speed_scale=$(ros2 param get $NODE_NAME $prefix.normal.speed_scale_ratio 2>/dev/null | grep -oE '[-+]?[0-9]*\.?[0-9]+' || echo "1.0")
    
    # 3. 値の入力フォーム
    results=$(dialog --backtitle "Ackermann Filter Live Tuner" \
           --title "Live Tuning (Section $sec_num)" \
           --form "\nUpdating parameters for Section $sec_num" 15 60 2 \
           "Steer Offset:" 1 1 "$current_offset" 1 20 10 5 \
           "Speed Scale:"  2 1 "$current_speed_scale" 2 20 10 5 \
           2>&1 >/dev/tty)

    # Cancelが押されたら何もしないでセクション選択に戻る
    if [ $? -ne 0 ]; then
        continue
    fi

    # 4. パラメータのセット (ros2 param set)
    new_offset=$(echo "$results" | sed -n '1p')
    new_speed_scale=$(echo "$results" | sed -n '2p')
    
    dialog --title "Applying..." --infobox "\nSetting parameters for Section $sec_num...\n\nOffset: $new_offset\nSpeed Scale: $new_speed_scale" 8 50
    
    # ★ ここで ros2 param set を実行
    ros2 param set $NODE_NAME $prefix.steer_offset $new_offset
    ros2 param set $NODE_NAME $prefix.normal.speed_scale_ratio $new_speed_scale
    
    # ノード側（ターミナル1）のログに、パラメータ変更が適用された旨の
    # デバッグプリントが出るはずです。
    sleep 0.5 # infoboxを少し表示
done

clear
echo "Live tuning finished."
echo "注意: 変更は実行中のノードにのみ適用されています。"
echo "設定を永続化(保存)するには、以下のコマンドでYAMLにダンプできます:"
echo "ros2 param dump $NODE_NAME > my_tuned_params.yaml"