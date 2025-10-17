import torch
import argparse
from pathlib import Path

try:
    from src.model.trajformer import TrajFormer
except ImportError:
    print("="*50)
    print("[ERROR] 'from src.model.trajformer import TrajFormer' が失敗しました。")
    print("このスクリプトは、train.py と同じ階層（srcフォルダが見える場所）から実行してください。")
    print("="*50)
    exit(1)


def main(args):
    """TrajFormerモデルをONNX形式に変換します。"""

    checkpoint_path = Path(args.checkpoint).resolve()

    # --- 出力パスの決定ロジック ---
    if args.output:
        # --output が指定された場合は、そのパスを使用
        output_path = Path(args.output).resolve()
    else:
        # --output が指定されない場合、チェックポイントと同じ場所/名前で拡張子を .onnx に変更
        output_path = checkpoint_path.parent / f"{checkpoint_path.stem}.onnx"
    # ---

    print("--- Configuration ---")
    print(f"Checkpoint Path: {checkpoint_path}")
    print(f"Output ONNX Path: {output_path}") # 決定された出力パスを表示
    print(f"Input Image Shape: (1, 3, {args.height}, {args.width})")
    print(f"Input Odom Shape:  (1, {args.past_len}, {args.odom_dim})")
    print("---------------------")

    # 出力先のディレクトリが存在しない場合は作成
    output_path.parent.mkdir(parents=True, exist_ok=True)

    # 1. モデルをロードし、評価モードにする
    model = TrajFormer(
        history_len=args.past_len,
        odom_features=args.odom_dim,
        future_len=args.future_len,
        image_embedding_dim=args.image_embed_dim,
        motion_embedding_dim=args.motion_embed_dim,
        transformer_d_model=args.d_model,
        transformer_nhead=args.nhead,
        transformer_num_layers=args.num_layers
    )
    
    # CPUにマップして状態をロード (checkpoint_path を使用)
    model.load_state_dict(torch.load(checkpoint_path, map_location='cpu')) 
    model.eval()
    print("✅ Model TrajFormer loaded successfully.")

    # 2. ONNXエクスポート用のダミー入力データを作成
    dummy_input_image = torch.randn(1, 3, args.height, {args.width})
    dummy_input_odoms = torch.randn(1, args.past_len, args.odom_dim)

    # 3. ONNXとしてエクスポート
    try:
        print("[INFO] Starting ONNX export...")
        
        dynamic_shapes = {
            "image": {0: "batch_size"},
            "past_odoms": {0: "batch_size"}
        }

        torch.onnx.export(
            model,
            (dummy_input_image, dummy_input_odoms),
            str(output_path), # 決定された出力パスを使用
            
            input_names=['input_image', 'input_odoms'], 
            output_names=['output_trajectory'],
            
            opset_version=18,
            do_constant_folding=False, 
            dynamic_shapes=dynamic_shapes,
            optimize=False 
        )

        print(f"✅ ONNX export complete: {output_path}") 
        
    except Exception as e:
        print(f"❌ Error during ONNX export: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Export a trained TrajFormer model to ONNX format.")

    # --- 基本的な引数 ---
    parser.add_argument(
        '-c', '--checkpoint',
        type=str,
        required=True,
        help="[REQUIRED] Path to the trained model checkpoint (.pth file)."
    )
    
    # --- 出力引数の修正 ---
    parser.add_argument(
        '-o', '--output',
        type=str,
        default=None, 
        help="Path to save the output ONNX model. (Default: Same directory and basename as checkpoint)"
    )
    # ---

    # --- 入力形状に関する引数 (Dataset/Configと一致させる) ---
    parser.add_argument(
        '--height',
        type=int,
        default=120,
        help="Image height (Default: 120)"
    )
    parser.add_argument(
        '--width',
        type=int,
        default=160,
        help="Image width (Default: 160)"
    )
    parser.add_argument(
        '--past-len',
        type=int,
        default=5,
        help="Number of past odometry steps (history_len) (Default: 5)"
    )
    parser.add_argument(
        '--odom-dim',
        type=int,
        default=8,
        help="Dimension of odometry features (odom_features) (Default: 8)"
    )
    parser.add_argument(
        '--future-len',
        type=int,
        default=10,
        help="Number of future trajectory points to predict (future_len) (Default: 30)"
    )

    # --- モデルアーキテクチャに関する引数 (Configと一致させる) ---
    parser.add_argument('--image-embed-dim', type=int, default=128, help="Default: 128")
    parser.add_argument('--motion-embed-dim', type=int, default=64, help="Default: 64")
    parser.add_argument('--d-model', type=int, default=192, help="Transformer d_model (Default: 192)")
    parser.add_argument('--nhead', type=int, default=6, help="Transformer nhead (Default: 6)")
    parser.add_argument('--num-layers', type=int, default=2, help="Transformer num_layers (Default: 2)")
    
    args = parser.parse_args()
    main(args)