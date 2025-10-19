import torch
import argparse
from pathlib import Path
from collections import OrderedDict
import os

def merge_weights(traj_path: Path, ctrl_path: Path, output_path: Path):
    """
    TrajFormer と ControlNet の state_dict をマージして単一ファイルに保存する
    """
    
    # --- 1. 入力ファイルの存在チェック ---
    if not traj_path.exists():
        print(f"❌ エラー: TrajFormer の重みファイルが見つかりません: {traj_path}")
        return
    if not ctrl_path.exists():
        print(f"❌ エラー: ControlNet の重みファイルが見つかりません: {ctrl_path}")
        return

    # --- 2. 出力ディレクトリの準備 ---
    output_path.parent.mkdir(parents=True, exist_ok=True)

    # --- 3. 重みのロード (CPU推奨) ---
    print(f"🔄 Loading TrajFormer weights from: {traj_path}")
    # map_location='cpu' を指定することで、GPUがない環境でも実行可能
    traj_weights = torch.load(traj_path, map_location='cpu')
    
    print(f"🔄 Loading ControlNet weights from: {ctrl_path}")
    ctrl_weights = torch.load(ctrl_path, map_location='cpu')

    # --- 4. 新しい state_dict の作成 ---
    # TrajControlFormer の 'all' モードのキー構造に合わせる
    
    print("\nMerging weights...")
    merged_state_dict = OrderedDict()
    
    # 4a. TrajFormer のキーに 'trajformer.' プレフィックスを追加
    num_traj_keys = 0
    for key, value in traj_weights.items():
        new_key = f"trajformer.{key}"
        merged_state_dict[new_key] = value
        num_traj_keys += 1
    print(f"  > Added {num_traj_keys} keys from TrajFormer (prefix: 'trajformer.')")

    # 4b. ControlNet のキーに 'control_net.' プレフィックスを追加
    num_ctrl_keys = 0
    for key, value in ctrl_weights.items():
        new_key = f"control_net.{key}"
        merged_state_dict[new_key] = value
        num_ctrl_keys += 1
    print(f"  > Added {num_ctrl_keys} keys from ControlNet (prefix: 'control_net.')")

    # --- 5. マージした重みを保存 ---
    total_keys = len(merged_state_dict)
    if total_keys != (num_traj_keys + num_ctrl_keys):
        print("⚠️ キーの数に不一致があります (重複の可能性？)。")
        
    print(f"\nTotal keys in merged model: {total_keys}")

    try:
        torch.save(merged_state_dict, output_path)
        print(f"\n✅ Successfully saved merged model to: {output_path}")
    except Exception as e:
        print(f"\n❌ Failed to save merged model: {e}")

# --- main (コマンドライン引数の処理) ---
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Merge separately trained TrajFormer and ControlNet weights into a single file for 'all' mode."
    )
    
    parser.add_argument(
        "--traj_path", 
        type=str, 
        required=True, 
        help="Path to the trained TrajFormer weights (e.g., best_trajformer.pth)"
    )
    
    parser.add_argument(
        "--ctrl_path", 
        type=str, 
        required=True, 
        help="Path to the trained ControlNet weights (e.g., best_controlformer.pth)"
    )
    
    parser.add_argument(
        "--output_path", 
        type=str, 
        required=True, 
        help="Path to save the merged model weights (e.g., merged_model.pth)"
    )

    args = parser.parse_args()
    
    merge_weights(
        traj_path=Path(args.traj_path),
        ctrl_path=Path(args.ctrl_path),
        output_path=Path(args.output_path)
    )