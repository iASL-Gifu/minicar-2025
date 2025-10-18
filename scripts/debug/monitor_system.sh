#!/bin/bash

# Jetson システム監視 - 記録モード / 表示モード切り替え
# 使用方法:
#   ./monitor_system.sh record <計測秒数>     - 記録のみ（バックグラウンド向け）
#   ./monitor_system.sh display <計測秒数>    - 画面表示（デフォルト）
#   ./monitor_system.sh record                 - 無限記録（Ctrl+Cで停止）
#   ./monitor_system.sh display                - 無限表示（Ctrl+Cで停止）
#   ./monitor_system.sh 60                     - 表示モード（デフォルト）
#   ./monitor_system.sh                        - 無限表示モード

MODE=${1:-display}
DURATION=${2:-0}

# モード判定
if [ "$MODE" != "record" ] && [ "$MODE" != "display" ]; then
  if [[ "$MODE" =~ ^[0-9]+$ ]]; then
    DURATION=$MODE
    MODE="display"
  else
    echo "Usage:"
    echo "  ./monitor_system.sh display <seconds>  - Display mode (default)"
    echo "  ./monitor_system.sh record <seconds>   - Record only (silent)"
    echo "  ./monitor_system.sh <seconds>          - Display mode"
    echo "  ./monitor_system.sh                    - Infinite display mode (Ctrl+C to stop)"
    exit 1
  fi
fi

OUTPUT_FILE="/debug/system_monitor_$(date +%Y%m%d_%H%M%S).csv"

# ヘッダー作成
echo "timestamp,cpu_percent,ram_percent,swap_percent,ram_used_mb,ram_total_mb,swap_used_mb,swap_total_mb" > "$OUTPUT_FILE"

# Ctrl+C での終了処理
trap 'echo -e "\n\nMonitoring stopped. Data saved to: $OUTPUT_FILE"; exit 0' INT

# ===== CPU使用率取得関数 =====
get_cpu_usage() {
  if command -v mpstat >/dev/null 2>&1; then
    CPU_PERCENT=$(mpstat 1 1 | awk '/Average/ && $12 ~ /[0-9.]+/ {printf "%.1f", 100 - $12}')
  else
    CPU_PERCENT=$(top -bn1 | grep -E "Cpu\(s\)|Cpu " | head -n 1 | \
      sed "s/.*, *\([0-9.]*\)%* id.*/\1/" | awk '{printf "%.1f", 100 - $1}')
  fi
  echo "$CPU_PERCENT"
}

# ===== 記録モード =====
if [ "$MODE" = "record" ]; then
  if [ "$DURATION" -eq 0 ]; then
    echo "Recording mode: Infinite (Press Ctrl+C to stop) -> $OUTPUT_FILE" >&2
  else
    echo "Recording mode: $DURATION seconds -> $OUTPUT_FILE" >&2
  fi
  
  i=1
  while true; do
    TIMESTAMP=$(date "+%Y-%m-%d %H:%M:%S")
    
    CPU_PERCENT=$(get_cpu_usage)
    
    # RAM
    RAM_TOTAL=$(free | grep Mem | awk '{print $2}')
    RAM_USED=$(free | grep Mem | awk '{print $3}')
    RAM_PERCENT=$(awk "BEGIN {printf \"%.1f\", ($RAM_USED / $RAM_TOTAL) * 100}")
    RAM_USED_MB=$((RAM_USED / 1024))
    RAM_TOTAL_MB=$((RAM_TOTAL / 1024))
    
    # SWAP
    SWAP_TOTAL=$(free | grep Swap | awk '{print $2}')
    SWAP_USED=$(free | grep Swap | awk '{print $3}')
    if [ "$SWAP_TOTAL" -gt 0 ]; then
      SWAP_PERCENT=$(awk "BEGIN {printf \"%.1f\", ($SWAP_USED / $SWAP_TOTAL) * 100}")
    else
      SWAP_PERCENT="0.0"
    fi
    SWAP_USED_MB=$((SWAP_USED / 1024))
    SWAP_TOTAL_MB=$((SWAP_TOTAL / 1024))
    
    # CSV記録
    echo "$TIMESTAMP,$CPU_PERCENT,$RAM_PERCENT,$SWAP_PERCENT,$RAM_USED_MB,$RAM_TOTAL_MB,$SWAP_USED_MB,$SWAP_TOTAL_MB" >> "$OUTPUT_FILE"
    
    # 秒数指定がある場合は終了判定
    if [ "$DURATION" -gt 0 ] && [ "$i" -ge "$DURATION" ]; then
      break
    fi
    
    i=$((i + 1))
    sleep 1
  done
  
  echo "Done: $OUTPUT_FILE" >&2
  exit 0
fi

# ===== 表示モード =====
clear

echo "╔══════════════════════════════════════════════════════╗"
echo "║     Jetson System Monitor - Total Usage            ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""

CPU_CORES=$(nproc)
MEM_TOTAL=$(free | grep Mem | awk '{print $2}')
MEM_TOTAL_MB=$((MEM_TOTAL / 1024))

echo "System Info"
echo "  CPU Cores: $CPU_CORES"
echo "  Total RAM: ${MEM_TOTAL_MB} MB"
if [ "$DURATION" -eq 0 ]; then
  echo "  Duration: Infinite (Press Ctrl+C to stop)"
else
  echo "  Duration: ${DURATION} sec"
fi
echo "  Output File: $OUTPUT_FILE"
echo ""

i=1
while true; do
  TIMESTAMP=$(date "+%Y-%m-%d %H:%M:%S")
  ELAPSED=$((i))
  
  CPU_PERCENT=$(get_cpu_usage)
  
  FREE_OUTPUT=$(free)
  RAM_TOTAL=$(echo "$FREE_OUTPUT" | grep Mem | awk '{print $2}')
  RAM_USED=$(echo "$FREE_OUTPUT" | grep Mem | awk '{print $3}')
  RAM_PERCENT=$(awk "BEGIN {printf \"%.1f\", ($RAM_USED / $RAM_TOTAL) * 100}")
  RAM_USED_MB=$((RAM_USED / 1024))
  RAM_TOTAL_MB=$((RAM_TOTAL / 1024))
  
  SWAP_TOTAL=$(echo "$FREE_OUTPUT" | grep Swap | awk '{print $2}')
  SWAP_USED=$(echo "$FREE_OUTPUT" | grep Swap | awk '{print $3}')
  if [ "$SWAP_TOTAL" -gt 0 ]; then
    SWAP_PERCENT=$(awk "BEGIN {printf \"%.1f\", ($SWAP_USED / $SWAP_TOTAL) * 100}")
  else
    SWAP_PERCENT="0.0"
  fi
  SWAP_USED_MB=$((SWAP_USED / 1024))
  SWAP_TOTAL_MB=$((SWAP_TOTAL / 1024))
  
  # CSV出力
  echo "$TIMESTAMP,$CPU_PERCENT,$RAM_PERCENT,$SWAP_PERCENT,$RAM_USED_MB,$RAM_TOTAL_MB,$SWAP_USED_MB,$SWAP_TOTAL_MB" >> "$OUTPUT_FILE"
  
  # プログレスバー作成
  CPU_INT=$(printf "%.0f" "$CPU_PERCENT")
  RAM_INT=$(printf "%.0f" "$RAM_PERCENT")
  SWAP_INT=$(printf "%.0f" "$SWAP_PERCENT")
  
  CPU_BAR=$(printf '#%.0s' $(seq 1 $((CPU_INT / 5))))
  RAM_BAR=$(printf '#%.0s' $(seq 1 $((RAM_INT / 5))))
  SWAP_BAR=$(printf '#%.0s' $(seq 1 $((SWAP_INT / 5))))
  
  CPU_BAR=$(printf "%-20s" "$CPU_BAR")
  RAM_BAR=$(printf "%-20s" "$RAM_BAR")
  SWAP_BAR=$(printf "%-20s" "$SWAP_BAR")
  
  clear
  echo "╔══════════════════════════════════════════════════════╗"
  echo "║     Jetson System Monitor - Total Usage            ║"
  echo "╚══════════════════════════════════════════════════════╝"
  echo ""
  if [ "$DURATION" -eq 0 ]; then
    echo "Elapsed: $ELAPSED sec (Press Ctrl+C to stop)"
  else
    echo "Elapsed: $ELAPSED / $DURATION sec"
  fi
  echo ""
  echo "CPU Usage"
  echo "  [$CPU_BAR] ${CPU_PERCENT}%"
  echo ""
  echo "RAM Usage"
  echo "  [$RAM_BAR] ${RAM_PERCENT}%"
  echo "  Used: ${RAM_USED_MB} MB / Total: ${RAM_TOTAL_MB} MB"
  echo ""
  echo "SWAP Usage"
  echo "  [$SWAP_BAR] ${SWAP_PERCENT}%"
  echo "  Used: ${SWAP_USED_MB} MB / Total: ${SWAP_TOTAL_MB} MB"
  echo ""
  echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
  echo "Time: $TIMESTAMP"
  
  if [ "$DURATION" -gt 0 ] && [ "$i" -ge "$DURATION" ]; then
    break
  fi
  
  i=$((i + 1))
  sleep 1
done

echo ""
echo "Saved: $OUTPUT_FILE"
