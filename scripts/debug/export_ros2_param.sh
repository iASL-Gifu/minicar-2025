#!/bin/bash

# 使用方法: ./export_ros2_params.sh
# ノードをインタラクティブに選択してパラメータをエクスポート

# 色付き出力用
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 保存先ディレクトリ
DEBUG_DIR="/debug"

# /debugディレクトリが存在するか確認
if [ ! -d "${DEBUG_DIR}" ]; then
    echo -e "${YELLOW}Creating ${DEBUG_DIR} directory...${NC}"
    if ! mkdir -p "${DEBUG_DIR}" 2>/dev/null; then
        echo -e "${RED}Error: Cannot create ${DEBUG_DIR} (try with sudo)${NC}"
        exit 1
    fi
fi

# /debugへの書き込み権限を確認
if [ ! -w "${DEBUG_DIR}" ]; then
    echo -e "${RED}Error: No write permission for ${DEBUG_DIR}${NC}"
    exit 1
fi

echo -e "${GREEN}=== インタラクティブ ROS2 パラメータエクスポーター ===${NC}\n"

# ノードリストを取得
node_list=$(ros2 node list 2>/dev/null)

if [ -z "$node_list" ]; then
    echo -e "${RED}Error: No nodes found or ros2 is not available${NC}"
    exit 1
fi

# ノードを配列に変換
mapfile -t nodes <<< "$node_list"

# ノード数が0の場合
if [ ${#nodes[@]} -eq 0 ]; then
    echo -e "${RED}Error: No nodes found${NC}"
    exit 1
fi

# ノードを表示
echo -e "${BLUE}Available nodes:${NC}"
for i in "${!nodes[@]}"; do
    printf "%3d) %s\n" $((i + 1)) "${nodes[$i]}"
done

# ユーザーに選択を促す
echo ""
while true; do
    read -p "Select node number (1-${#nodes[@]}): " selection
    
    # 入力が整数か確認
    if ! [[ "$selection" =~ ^[0-9]+$ ]]; then
        echo -e "${RED}Error: Please enter a valid number${NC}"
        continue
    fi
    
    # 範囲チェック
    if [ "$selection" -lt 1 ] || [ "$selection" -gt ${#nodes[@]} ]; then
        echo -e "${RED}Error: Number out of range. Please select 1-${#nodes[@]}${NC}"
        continue
    fi
    
    break
done

# 選択したノードを取得
NODE_NAME="${nodes[$((selection - 1))]}"

# ファイル名を生成（ノード名から）
# スラッシュを削除して、アンダースコアに置き換える
NODE_CLEAN=$(echo "${NODE_NAME}" | sed 's/^\///' | sed 's/\//_/g')
OUTPUT_FILE="${DEBUG_DIR}/${NODE_CLEAN}_params.yaml"

echo -e "\n${GREEN}Selected Node: ${YELLOW}${NODE_NAME}${NC}"
echo -e "${GREEN}Output File: ${YELLOW}${OUTPUT_FILE}${NC}\n"

# 一時ファイルを作成
TEMP_FILE=$(mktemp)

# 中断時にも保存できるようにトラップを設定
trap 'echo -e "\n${YELLOW}Interrupted! Saving partial results...${NC}"; cp "${TEMP_FILE}" "${OUTPUT_FILE}" 2>/dev/null; rm -f "${TEMP_FILE}"; exit 130' INT TERM

# YAMLヘッダーを作成
cat > "${TEMP_FILE}" << EOF
# ROS2 Parameters for node: ${NODE_NAME}
# Generated on: $(date)
# Command: ros2 param dump ${NODE_NAME}

${NODE_NAME}:
  ros__parameters:
EOF

# パラメータリストを取得
param_list=$(ros2 param list "${NODE_NAME}" 2>/dev/null)

# 各パラメータの値を取得
echo -e "${YELLOW}Exporting parameters...${NC}"
count=0
total=$(echo "$param_list" | wc -l)

while IFS= read -r param; do
    if [ -n "$param" ]; then
        count=$((count + 1))
        echo -ne "\rProgress: ${count}/${total}"
        
        # パラメータ名の先頭のスペースを削除
        param_clean=$(echo "$param" | sed 's/^[[:space:]]*//')
        
        # パラメータの値を取得（クリーンな名前を使用）
        value=$(ros2 param get "${NODE_NAME}" "${param_clean}" 2>/dev/null)
        
        if [ $? -eq 0 ]; then
            # 値を抽出（複数の形式に対応）
            actual_value=""
            
            # Array/List形式: "String values are: ['value1', 'value2']"
            if echo "$value" | grep -q "values are:"; then
                actual_value=$(echo "$value" | sed -n "s/.*values are: //p" | xargs)
            # 単一値形式: "String value is: text" または "Integer value is: 30"
            elif echo "$value" | grep -q "value is:"; then
                actual_value=$(echo "$value" | sed -n 's/.*value is: //p' | xargs)
            # 古い形式 "Value: X"
            elif echo "$value" | grep -q "^Value:"; then
                actual_value=$(echo "$value" | grep "^Value:" | sed 's/^Value: //')
            else
                # その他の形式：最後の行を取得
                actual_value=$(echo "$value" | tail -1 | xargs)
            fi
            
            # 値の型に応じて適切にフォーマット
            if [[ "$actual_value" == "true" ]] || [[ "$actual_value" == "false" ]] || [[ "$actual_value" == "True" ]] || [[ "$actual_value" == "False" ]]; then
                # Boolean値（小文字に変換）
                actual_value_lower=$(echo "$actual_value" | tr '[:upper:]' '[:lower:]')
                echo "    ${param_clean}: ${actual_value_lower}" >> "${TEMP_FILE}"
            elif [[ "$actual_value" =~ ^-?[0-9]+$ ]]; then
                # 整数値
                echo "    ${param_clean}: ${actual_value}" >> "${TEMP_FILE}"
            elif [[ "$actual_value" =~ ^-?[0-9]+\.[0-9]+$ ]]; then
                # 浮動小数点値
                echo "    ${param_clean}: ${actual_value}" >> "${TEMP_FILE}"
            elif [[ "$actual_value" == "["* ]]; then
                # リスト/配列
                echo "    ${param_clean}: ${actual_value}" >> "${TEMP_FILE}"
            else
                # 文字列値（クォートで囲む）
                echo "    ${param_clean}: '${actual_value}'" >> "${TEMP_FILE}"
            fi
        fi
    fi
done <<< "$param_list"

# 一時ファイルを最終出力先にコピー
cp "${TEMP_FILE}" "${OUTPUT_FILE}"

# 一時ファイルを削除
rm -f "${TEMP_FILE}"

echo -e "\n${GREEN}✓ Successfully exported ${count} parameters to ${OUTPUT_FILE}${NC}"
echo -e "\nYou can load these parameters with:"
echo -e "${YELLOW}ros2 param load ${NODE_NAME} ${OUTPUT_FILE}${NC}"