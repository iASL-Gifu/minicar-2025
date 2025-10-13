import torch
import argparse
from pathlib import Path

from src.model.pilotnet import PilotNet

def main(args):
    """PyTorchモデルをONNX形式に変換します。"""

    print("--- Configuration ---")
    print(f"Checkpoint Path: {args.checkpoint}")
    print(f"Output ONNX Path: {args.output}")
    print(f"Input Shape: (1, 3, {args.height}, {args.width})")
    print("---------------------")

    # 出力先のディレクトリが存在しない場合は作成
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    # 1. モデルをロードし、評価モードにする
    model = PilotNet(num_outputs=2)
    model.load_state_dict(torch.load(args.checkpoint, map_location='cpu')) 
    model.eval()
    print("✅ Model loaded successfully.")

    # 2. ONNXエクスポート用のダミー入力データを作成
    dummy_input = torch.randn(1, 3, args.height, args.width)

    # 3. ONNXとしてエクスポート
    try:
        torch.onnx.export(
            model,
            dummy_input,
            str(output_path),         # 出力ファイル名
            input_names=['input_1'],  # Tritonのconfig.pbtxtで使う入力名
            output_names=['output_1'],# Tritonのconfig.pbtxtで使う出力名
            opset_version=12,
            dynamic_axes={
                'input_1': {0: 'batch_size'}, 
                'output_1': {0: 'batch_size'}
            }
        )
        print(f"✅ ONNX export complete: {args.output}")
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
        '-o', '--output',
        type=str,
        default="onnx_models/pilotnet.onnx",
        help="Path to save the output ONNX model. (Default: onnx_models/pilotnet.onnx)"
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