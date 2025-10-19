#!/bin/bash

# --- 設定 ---
PREPROCESS_SCRIPT_NAME="1_extract_temporal.py"

# スクリプト自身のディレクトリを取得
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
PYTHON_SCRIPT_PATH="${SCRIPT_DIR}/${PREPROCESS_SCRIPT_NAME}"

# --- 色付け (オプション) ---
CYAN='\033[0;36m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# --- ヘルプ関数 [MODIFIED] ---
show_help() {
    echo "Usage: $0 -b <path> -o <path>"
    echo ""
    echo "Interactively select sequences and split them into train/test datasets."
    echo ""
    echo "Options:"
    echo "  -b, --base_dir   Base directory to search for sequences (recursively)"
    echo "  -o, --outdir     Output root directory (e.g., ./datasets)"
    echo "  -h, --help       Show this help message"
}

# --- 引数解析 [MODIFIED] ---
BASE_DIR=""
OUTDIR=""

while [[ $# -gt 0 ]]; do
    key="$1"
    case $key in
        -b|--base_dir)
        BASE_DIR="$2"
        shift # past argument
        shift # past value
        ;;
        -o|--outdir)
        OUTDIR="$2"
        shift # past argument
        shift # past value
        ;;
        -h|--help)
        show_help
        exit 0
        ;;
        *)    # unknown option
        echo "Unknown option: $1"
        show_help
        exit 1
        ;;
    esac
done

if [ -z "$BASE_DIR" ] || [ -z "$OUTDIR" ]; then
    echo -e "${RED}ERROR: Both -b (or --base_dir) and -o (or --outdir) are required.${NC}"
    show_help
    exit 1
fi

# Pythonスクリプトの存在チェック
if [ ! -f "$PYTHON_SCRIPT_PATH" ]; then
    echo -e "${RED}CRITICAL ERROR: Preprocessing script not found at:${NC}"
    echo "  $PYTHON_SCRIPT_PATH"
    exit 1
fi

# --- 1. シーケンスを探索 ---
echo -e "🔄 Searching for sequences under: ${CYAN}$BASE_DIR${NC}"
mapfile -t sequences < <(find "$BASE_DIR" -name "metadata.yaml" -print0 | xargs -0 -I {} dirname {} | sort)

if [ ${#sequences[@]} -eq 0 ]; then
    echo -e "${YELLOW}❌ No sequences found.${NC}"
    exit 0
fi

# --- 2. シーケンスを表示 ---
echo -e "\n--- 📂 Found Sequences ---"
for i in "${!sequences[@]}"; do
    relative_path=$(echo "${sequences[$i]}" | sed "s|^${BASE_DIR}/||")
    printf "  [${GREEN}%02d${NC}] %s\n" "$((i+1))" "$relative_path"
done
echo -e "----------------------------\n"

# --- 3. 選択関数 ---
select_sequences_bash() {
    local prompt_message="$1"
    local -n output_array=$2
    
    echo -e "👉 ${CYAN}$prompt_message${NC}"
    echo "   (Enter numbers separated by space, e.g., 1 3 5)"
    read -p "   Select: " -a indices

    output_array=()
    for idx in "${indices[@]}"; do
        if [[ "$idx" =~ ^[0-9]+$ ]] && [ "$idx" -ge 1 ] && [ "$idx" -le "${#sequences[@]}" ]; then
            output_array+=("${sequences[$((idx-1))]}")
        else
            echo -e "   ${YELLOW}⚠️ Invalid number skipped: $idx${NC}"
        fi
    done
    
    echo "   Selected:"
    for p in "${output_array[@]}"; do
        echo -e "     ${GREEN}✅ $(basename "$p")${NC}"
    done
    echo ""
}

# --- 4. 実行関数 ---
run_extraction_bash() {
    local output_dir="$1"
    shift
    local seq_paths=("$@")
    
    if [ ${#seq_paths[@]} -eq 0 ]; then
        echo -e "ℹ️ No sequences selected for $(basename "$output_dir"). Skipping."
        return
    fi

    mkdir -p "$output_dir"
    echo -e "\n🚀 Starting extraction for ${GREEN}$(basename "$output_dir")${NC} dataset..."
    echo -e "   Outputting to: ${CYAN}$output_dir${NC}"

    python3 "$PYTHON_SCRIPT_PATH" --seq_dirs "${seq_paths[@]}" --outdir "$output_dir"
    
    if [ $? -eq 0 ]; then
        echo -e "✅ Finished extraction for ${GREEN}$(basename "$output_dir")${NC}."
    else
        echo -e "${RED}❌ ERROR: Extraction failed for $(basename "$output_dir").${NC}"
    fi
}

# --- 5. メイン処理 ---
declare -a train_paths
declare -a test_paths

select_sequences_bash "Select TRAIN sequences." train_paths
select_sequences_bash "Select TEST sequences." test_paths

run_extraction_bash "$OUTDIR/train" "${train_paths[@]}"
run_extraction_bash "$OUTDIR/test" "${test_paths[@]}"

echo -e "\n🎉 All tasks finished."