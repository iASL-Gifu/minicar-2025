#!/bin/bash

# ===== 設定 =====
DEFAULT_IP_TAMIYA="10.42.0.1"
DEFAULT_IP_TRAXXAS="192.168.11.14"

LOCAL_BASE_DIR="/workspaces/src/launch/localization_launch"
REMOTE_PATH_TAMIYA="/home/tamiya/workspace/minicar-2025/ros2_ws/src/launch/localization_launch"
REMOTE_PATH_TRAXXAS="/home/traxxas/minicar-2025/ros2_ws/src/launch/localization_launch"

# ===== ユーザー選択 =====
echo "送信先ユーザーを選択してください:"
select USER in "tamiya" "traxxas"; do
  case $USER in
    tamiya)
      DEFAULT_IP=$DEFAULT_IP_TAMIYA
      REMOTE_PATH=$REMOTE_PATH_TAMIYA
      break
      ;;
    traxxas)
      DEFAULT_IP=$DEFAULT_IP_TRAXXAS
      REMOTE_PATH=$REMOTE_PATH_TRAXXAS
      break
      ;;
    *)
      echo "無効な選択です。もう一度入力してください。"
      ;;
  esac
done

# ===== IP指定 =====
read -p "接続先IPアドレスを入力してください（デフォルト: $DEFAULT_IP）: " INPUT_IP
IP=${INPUT_IP:-$DEFAULT_IP}
echo "接続先IP: $IP"

# ===== パスワード入力 =====
read -s -p "パスワードを入力してください: " PASSWORD
echo ""

# ===== keyframesサブディレクトリ選択 =====
echo "keyframes ディレクトリ内のサブディレクトリを選択してください:"
select KF_SUBDIR in $(ls -d ${LOCAL_BASE_DIR}/keyframes/*/ 2>/dev/null | xargs -n1 basename); do
  if [ -d "${LOCAL_BASE_DIR}/keyframes/$KF_SUBDIR" ]; then
    echo "選択された keyframes: $KF_SUBDIR"
    break
  else
    echo "無効な選択です。"
  fi
done

# ===== mapサブディレクトリ選択 =====
echo "map ディレクトリ内のサブディレクトリを選択してください:"
select MAP_SUBDIR in $(ls -d ${LOCAL_BASE_DIR}/map/*/ 2>/dev/null | xargs -n1 basename); do
  if [ -d "${LOCAL_BASE_DIR}/map/$MAP_SUBDIR" ]; then
    echo "選択された map: $MAP_SUBDIR"
    break
  else
    echo "無効な選択です。"
  fi
done

# ===== 確認 =====
echo
echo "===== 送信内容確認 ====="
echo "送信先: ${USER}@${IP}:${REMOTE_PATH}"
echo "keyframes: ${KF_SUBDIR}"
echo "map: ${MAP_SUBDIR}"
echo "ファイル: section_detector.yaml"
echo "========================="
read -p "この内容で送信しますか？ (y/n): " CONFIRM
[ "$CONFIRM" != "y" ] && echo "キャンセルしました。" && exit 0

# ===== SCP送信 =====
sshpass -p "$PASSWORD" scp -v -C -r "${LOCAL_BASE_DIR}/keyframes/${KF_SUBDIR}" "${USER}@${IP}:${REMOTE_PATH}/keyframes/"
sshpass -p "$PASSWORD" scp -v -C -r "${LOCAL_BASE_DIR}/map/${MAP_SUBDIR}" "${USER}@${IP}:${REMOTE_PATH}/map/"
sshpass -p "$PASSWORD" scp -v -C "${LOCAL_BASE_DIR}/config/section_detector.yaml" "${USER}@${IP}:${REMOTE_PATH}/config/"

echo "✅ 送信完了"
