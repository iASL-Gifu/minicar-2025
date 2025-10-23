#!/bin/bash

# --- このスクリプトについて ---
# 1. `create_dataset.sh` を実行して、データセットを作成します。
# 2. `train.sh` を実行して、作成したデータセットでモデルを学習します。

# --- 色付け (オプション) ---
CYAN='\033[0;36m'
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# --- ヘルプ関数 ---
show_help() {
    echo "Usage: $0 -b <path> -o <path>"
    echo ""
    echo "Runs the full pipeline: creates the dataset and then starts training."
    echo ""
    echo "Options:"
    echo "  -b, --base_dir      Base directory for sequences (passed to create_dataset.sh)"
    echo "  -o, --outdir        Output root directory for datasets (passed to both scripts)"
    echo "  -h, --help          Show this help message"
}

# 引数が一つもない場合、またはヘルプが要求された場合にヘルプを表示
if [ $# -eq 0 ]; then
    show_help
    exit 0
fi
for arg in "$@"; do
    if [ "$arg" == "-h" ] || [ "$arg" == "--help" ]; then
        show_help
        exit 0
    fi
done

# --- 引数解析 ---
# -o の値は `train.sh` にも渡す必要があるため、ここで取得しておく
OUTDIR=""
for ((i=1; i<=$#; i++)); do
    if [ "${!i}" == "-o" ] || [ "${!i}" == "--outdir" ]; then
        j=$((i+1))
        OUTDIR="${!j}"
        break
    fi
done

if [ -z "$OUTDIR" ]; then
    echo -e "${RED}ERROR: -o (--outdir) is a required argument for the pipeline.${NC}"
    show_help
    exit 1
fi


# --- 1. データセット作成 ---
echo -e "--- ${GREEN}STEP 1: Creating Dataset${NC} ---"
bash ./create_dataset.sh "$@"
# "$@" を使うことで、このスクリプトに渡されたすべての引数をそのまま渡す

if [ $? -ne 0 ]; then
    echo -e "\n${RED}❌ Pipeline stopped because dataset creation failed.${NC}"
    exit 1
fi

echo -e "\n--- ${GREEN}STEP 1 Finished Successfully${NC} ---"

# --- 2. 学習 ---
# 仮想環境のアクティベート（存在する場合）
if [ -f ../env/bin/activate ]; then
    echo -e "📦  Activating virtual environment: ${CYAN}../env/bin/activate${NC}"
    source ../env/bin/activate
else
    echo -e "${YELLOW}⚠️  Virtual environment ../env/bin/activate not found. Continuing without venv.${NC}"
fi

echo -e "\n--- ${GREEN}STEP 2: Starting Training${NC} ---"
bash ./train.sh --data_path "$OUTDIR"

if [ $? -ne 0 ]; then
    echo -e "\n${RED}❌ Pipeline stopped because training failed.${NC}"
    exit 1
fi

echo -e "\n🎉🎉🎉 ${GREEN}Full pipeline finished successfully!${NC} 🎉🎉🎉"