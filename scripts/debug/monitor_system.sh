#!/bin/bash

# Jetson システム監視 - 記録モード / 表示モード切り替え
# 使用方法:
#   ./monitor_system.sh record <計測秒数>     - 記録のみ（バックグラウンド向け）
#   ./monitor_system.sh display <計測秒数>    - 画面表示（デフォルト）
#   ./monitor_system.sh 60                     - 表示モード（デフォルト）

MODE=${1:-display}
DURATION=${2:-60}

# モード判定
if [ "$MODE" != "record" ] && [ "$MODE" != "display" ]; then
    # 第1引数が数字の場合は表示モード
    if [[ "$MODE" =~ ^[0-9]+$ ]]; then
        DURATION=$MODE
        MODE="display"
    else
        echo "Usage:"
        echo "  ./monitor_system.sh display <seconds>  - Display mode (default)"
        echo "  ./monitor_system.sh record <seconds>   - Record only (silent)"
        echo "  ./monitor_system.sh <seconds>          - Display mode"
        exit 1
    fi
fi

OUTPUT_FILE="/debug/system_monitor_$(date +%Y%m%d_%H%M%S).csv"

# ヘッダー作成
echo "timestamp,cpu_percent,ram_percent,swap_percent,ram_used_mb,ram_total_mb,swap_used_mb,swap_total_mb" > "$OUTPUT_FILE"

# 記録のみモード
if [ "$MODE" = "record" ]; then
    echo "Recording mode: $DURATION seconds -> $OUTPUT_FILE" >&2
    
    for i in $(seq 1 $DURATION); do
        TIMESTAMP=$(date "+%Y-%m-%d %H:%M:%S")
        
        # CPU
        CPU_PERCENT=$(top -bn1 | grep "Cpu(s)" | sed "s/.*, *\([0-9.]*\)%* id.*/\1/" | awk '{printf "%.1f", 100 - $1}')
        
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
        
        sleep 1
    done
    
    echo "Done: $OUTPUT_FILE" >&2
    exit 0
fi

# 表示モード
clear

echo "╔══════════════════════════════════════════════════════╗"
echo "║     Jetson System Monitor - Total Usage            ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""

# システム情報表示
CPU_CORES=$(nproc)
MEM_TOTAL=$(free | grep Mem | awk '{print $2}')
MEM_TOTAL_MB=$((MEM_TOTAL / 1024))

echo "System Info"
echo "  CPU Cores: $CPU_CORES"
echo "  Total RAM: ${MEM_TOTAL_MB} MB"
echo "  Duration: ${DURATION} sec"
echo "  Output File: $OUTPUT_FILE"
echo ""

# 計測開始
for i in $(seq 1 $DURATION); do
    TIMESTAMP=$(date "+%Y-%m-%d %H:%M:%S")
    ELAPSED=$((i))
    
    # CPU 使用率（全体）
    CPU_PERCENT=$(top -bn1 | grep "Cpu(s)" | sed "s/.*, *\([0-9.]*\)%* id.*/\1/" | awk '{printf "%.1f", 100 - $1}')
    
    # メモリ情報取得
    FREE_OUTPUT=$(free)
    
    # RAM情報
    RAM_TOTAL=$(echo "$FREE_OUTPUT" | grep Mem | awk '{print $2}')
    RAM_USED=$(echo "$FREE_OUTPUT" | grep Mem | awk '{print $3}')
    RAM_PERCENT=$(awk "BEGIN {printf \"%.1f\", ($RAM_USED / $RAM_TOTAL) * 100}")
    RAM_USED_MB=$((RAM_USED / 1024))
    RAM_TOTAL_MB=$((RAM_TOTAL / 1024))
    
    # SWAP情報
    SWAP_TOTAL=$(echo "$FREE_OUTPUT" | grep Swap | awk '{print $2}')
    SWAP_USED=$(echo "$FREE_OUTPUT" | grep Swap | awk '{print $3}')
    
    if [ "$SWAP_TOTAL" -gt 0 ]; then
        SWAP_PERCENT=$(awk "BEGIN {printf \"%.1f\", ($SWAP_USED / $SWAP_TOTAL) * 100}")
    else
        SWAP_PERCENT="0.0"
    fi
    
    SWAP_USED_MB=$((SWAP_USED / 1024))
    SWAP_TOTAL_MB=$((SWAP_TOTAL / 1024))
    
    # CSV に記録
    echo "$TIMESTAMP,$CPU_PERCENT,$RAM_PERCENT,$SWAP_PERCENT,$RAM_USED_MB,$RAM_TOTAL_MB,$SWAP_USED_MB,$SWAP_TOTAL_MB" >> "$OUTPUT_FILE"
    
    # プログレスバー作成 (CPU)
    CPU_INT=$(printf "%.0f" "$CPU_PERCENT")
    CPU_BAR=$(printf '#%.0s' $(seq 1 $((CPU_INT / 5))))
    CPU_BAR=$(printf "%-20s" "$CPU_BAR")
    
    # プログレスバー作成 (RAM)
    RAM_INT=$(printf "%.0f" "$RAM_PERCENT")
    RAM_BAR=$(printf '#%.0s' $(seq 1 $((RAM_INT / 5))))
    RAM_BAR=$(printf "%-20s" "$RAM_BAR")
    
    # プログレスバー作成 (SWAP)
    SWAP_INT=$(printf "%.0f" "$SWAP_PERCENT")
    SWAP_BAR=$(printf '#%.0s' $(seq 1 $((SWAP_INT / 5))))
    SWAP_BAR=$(printf "%-20s" "$SWAP_BAR")
    
    # 画面表示
    clear
    echo "╔══════════════════════════════════════════════════════╗"
    echo "║     Jetson System Monitor - Total Usage            ║"
    echo "╚══════════════════════════════════════════════════════╝"
    echo ""
    echo "Elapsed: $ELAPSED / $DURATION sec"
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
    
    sleep 1
done

# 計測完了
echo "Saved: $OUTPUT_FILE"