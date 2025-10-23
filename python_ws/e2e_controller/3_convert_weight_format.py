import torch
import argparse
from pathlib import Path

try:
    from src.model.pilotnet import PilotNet, TinyPilotNet
except ImportError:
    print("="*50)
    print("[ERROR] 'from src.model.pilotnet import PilotNet' が失敗しました。")
    print("このスクリプトは、train.py と同じ階層（srcフォルダが見える場所）から実行してください。")
    print("="*50)
    exit(1)

def main(args):
    """PyTorchモデルをONNX形式に変換します。"""

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
    print(f"Input Shape: (1, 3, {args.height}, {args.width})")
    print("---------------------")

    # 出力先のディレクトリが存在しない場合は作成
    output_path.parent.mkdir(parents=True, exist_ok=True)

    # 1. モデルをロードし、評価モードにする
    model = PilotNet(num_outputs=2) # PilotNetは引数が少ないので簡単
    model.load_state_dict(torch.load(checkpoint_path, map_location='cpu')) 
    model.eval()
    print("✅ Model loaded successfully.")

    # 2. ONNXエクスポート用のダミー入力データを作成
    dummy_input = torch.randn(1, 3, args.height, args.width)

    # 3. ONNXとしてエクスポート
    try:
        torch.onnx.export(
            model,
            dummy_input,
            str(output_path),         
            input_names=['input_1'],  
            output_names=['output_1'],
            opset_version=12,
            dynamic_axes={
                'input_1': {0: 'batch_size'}, 
                'output_1': {0: 'batch_size'}
            }
        )
        print(f"✅ ONNX export complete: {output_path}") 
    except Exception as e:
        print(f"❌ Error during ONNX export: {e}")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Export a trained PilotNet model to ONNX format.")

    # --- 引数の定義 ---
    parser.add_argument(
        '-c', '--checkpoint',
        type=str,
        required=True,
        help="[REQUIRED] Path to the trained model checkpoint (.pth file)."
    )

    parser.add_argument(
        '-s', '--size',
        type=str,   
        default='normal',
        help="Input image size. (Default: normal)"
    )
    
    parser.add_argument(
        '-o', '--output',
        type=str,
        default=None,
        help="Path to save the output ONNX model. (Default: Same directory and basename as checkpoint)"
    )

    parser.add_argument(
        '-H', '--height',
        type=int,
        default=120,
        help="Image height of the model input. (Default: 120)"
    )
    parser.add_argument(
        '-W', '--width',
        type=int,
        default=160,
        help="Image width of the model input. (Default: 160)"
    )
    
    args = parser.parse_args()
    main(args)