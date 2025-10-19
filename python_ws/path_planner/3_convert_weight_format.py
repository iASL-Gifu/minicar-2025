import torch
import argparse
from pathlib import Path

try:
    from src.model.trajcontrolnet import TrajControlFormer
except ImportError:
    print("="*50)
    print("[ERROR] 'from src.model.trajcontrolnet import TrajControlFormer' が失敗しました。")
    print("このスクリプトは、train.py と同じ階層（srcフォルダが見える場所）から実行してください。")
    print("="*50)
    exit(1)


def main(args):
    """TrajControlFormerモデルをONNX形式に変換します。"""

    checkpoint_path = Path(args.checkpoint).resolve()

    if args.output:
        output_path = Path(args.output).resolve()
    else:
        output_path = checkpoint_path.parent / f"{checkpoint_path.stem}.onnx"

    print("--- Configuration ---")
    print(f"Mode:              all (E2E)")
    print(f"Checkpoint Path:   {checkpoint_path}")
    print(f"Output ONNX Path:  {output_path}")
    print(f"Input Image Shape: (1, 3, {args.height}, {args.width})")
    print(f"Input Odom Shape:  (1, {args.past_len}, {args.odom_dim})")
    print("---------------------")

    output_path.parent.mkdir(parents=True, exist_ok=True)

    model = TrajControlFormer(
        # --- TrajFormer ---
        history_len=args.past_len,
        odom_features=args.odom_dim,
        future_len=args.future_len,
        image_embedding_dim=args.image_embed_dim,
        motion_embedding_dim=args.motion_embed_dim,
        transformer_d_model=args.d_model,
        transformer_nhead=args.nhead,
        transformer_num_layers=args.num_layers,
        
        # --- ControlFormer (独立GRU構成) ---
        control_motion_embedding_dim=args.control_motion_embed_dim,
        control_d_model=args.control_d_model,
        control_nhead=args.control_nhead,
        control_num_layers=args.control_num_layers
    )
    
    print("✅ Model TrajControlFormer shell created.")
    
    model.load_state_dict(torch.load(checkpoint_path, map_location='cpu'))
    model.eval()
    print("✅ Loaded full model weights.")

    dummy_input_image = torch.randn(1, 3, args.height, args.width)
    dummy_input_odoms = torch.randn(1, args.past_len, args.odom_dim)
    dummy_inputs = (dummy_input_image, dummy_input_odoms)
    
    input_names = ['input_image', 'input_odoms']
    output_names = ['output_trajectory', 'output_commands']
    
    dynamic_axes = {
        'input_image': {0: 'batch_size'},
        'input_odoms': {0: 'batch_size'},
        'output_trajectory': {0: 'batch_size'},
        'output_commands': {0: 'batch_size'}
    }

    try:
        print("[INFO] Starting ONNX export (mode: all)...")
        
        torch.onnx.export(
            model,
            dummy_inputs,
            str(output_path),
            
            input_names=input_names, 
            output_names=output_names,
            
            opset_version=18,
            do_constant_folding=True, 
            dynamic_axes=dynamic_axes,
            dynamo=False
        )

        print(f"✅ ONNX export complete: {output_path}") 
        
    except Exception as e:
        print(f"❌ Error during ONNX export: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Export a trained TrajControlFormer model to ONNX format.")

    # --- 基本的な引数 ---
    parser.add_argument(
        '-c', '--checkpoint',
        type=str,
        required=True,
        help="[REQUIRED] Path to the trained model checkpoint (.pth file)."
    )
    
    parser.add_argument(
        '-o', '--output',
        type=str,
        default=None, 
        help="Path to save the output ONNX model. (Default: Same directory and basename as checkpoint)"
    )
    
    # --- 入力形状に関する引数 (train.yaml と一致させる) ---
    parser.add_argument(
        '--height', type=int, default=120, help="Image height (Default: 120)"
    )
    parser.add_argument(
        '--width', type=int, default=160, help="Image width (Default: 160)"
    )
    parser.add_argument(
        '--past-len', type=int, default=10, help="Number of past odometry steps (history_len) (Default: 10)"
    )
    parser.add_argument(
        '--odom-dim', type=int, default=8, help="Dimension of odometry features (odom_features) (Default: 8)"
    )
    parser.add_argument(
        '--future-len', type=int, default=30, help="Number of future trajectory points (future_len) (Default: 30)"
    )
    
    # --- TrajFormer パラメータ ---
    parser.add_argument('--image-embed-dim', type=int, default=128, help="TrajFormer: Image Embedding Dim (Default: 128)")
    parser.add_argument('--motion-embed-dim', type=int, default=64, help="TrajFormer: GRU Embedding Dim (Default: 64)")
    parser.add_argument('--d-model', type=int, default=192, help="TrajFormer: Transformer d_model (Default: 192)")
    parser.add_argument('--nhead', type=int, default=6, help="TrajFormer: Transformer nhead (Default: 6)")
    parser.add_argument('--num-layers', type=int, default=2, help="TrajFormer: Transformer num_layers (Default: 2)")
    
    # --- ControlFormer パラメータ ---
    parser.add_argument('--control-motion-embed-dim', type=int, default=64, help="ControlFormer: GRU Embedding Dim (Default: 64)")
    parser.add_argument('--control-d-model', type=int, default=64, help="ControlFormer: Transformer d_model (Default: 64)")
    parser.add_argument('--control-nhead', type=int, default=4, help="ControlFormer: Transformer nhead (Default: 4)")
    parser.add_argument('--control-num-layers', type=int, default=2, help="ControlFormer: Transformer num_layers (Default: 2)")

    args = parser.parse_args()
    main(args)