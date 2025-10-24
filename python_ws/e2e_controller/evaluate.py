import torch
import torch.nn as nn
from torch.utils.data import DataLoader
from tqdm import tqdm
import hydra
from omegaconf import DictConfig, OmegaConf
import os
import numpy as np
import cv2  # OpenCVが必要です (pip install opencv-python)

# 学習スクリプトと同じ自作モジュールをインポート
from src.data.dataset import MultiSequenceDataset  
from src.data.transform import TestTransform # 推論時は TestTransform を使用
from src.model.pilotnet import PilotNet, TinyPilotNet

# =========================================================
# 【新しく追加】画像の最終的な拡大率を設定
# =========================================================
OUTPUT_SCALE_FACTOR = 2 # 例: 2倍に拡大 (2x)

def tensor_to_cv_image(tensor_image):
    """
    PyTorchテンソル [C, H, W] (0.0-1.0, RGB) を 
    OpenCV画像 [H, W, C] (0-255, BGR) に変換する
    
    (TestTransformが0-1のRGBテンソルを返すことを想定)
    """
    # [C, H, W] -> [H, W, C]
    img_np = tensor_image.cpu().numpy().transpose(1, 2, 0)
    
    # 0-1 の範囲を 0-255 にスケーリング
    img_np = (img_np * 255).astype(np.uint8)
    
    # RGB -> BGR (OpenCVの標準色空間)
    img_bgr = cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)
    return img_bgr

def draw_results_beside_image(image, steer_pred, steer_gt, speed_pred, speed_gt):
    """
    画像の横のスペースに推論結果とGTを描画する
    (最終出力は OUTPUT_SCALE_FACTOR で拡大されることを前提に、
     この関数内での描画は元のサイズで調整)
    """
    # 元の画像のサイズを取得
    h, w, c = image.shape
    
    # サイドパネルの幅を調整 (拡大後の表示を考慮して、より広めに)
    # ここは、拡大後に文字が全て収まるように調整してください
    side_panel_width = 150 # 元画像幅160px、パネル幅150px (合計310px)

    # 新しい画像の幅 = 元の画像の幅 + サイドパネルの幅
    new_w = w + side_panel_width
    
    # 新しい画像 (黒背景) を作成
    combined_image = np.zeros((h, new_w, c), dtype=np.uint8)
    
    # 元の画像を新しい画像の左側に配置
    combined_image[0:h, 0:w] = image

    # 描画設定 (OUTPUT_SCALE_FACTOR で拡大されるため、この時点では小さめ)
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.4 # 元のフォントスケール
    color_pred = (0, 255, 0)  # Green (予測)
    color_gt = (0, 0, 255)    # Red (GT)
    thickness = 1 # 元の線の太さ

    # テキスト描画 (X座標を元の画像の幅の右側から開始)
    text_x_start = w + 10 # 元の画像とテキストの間に10pxの隙間を空ける
    
    y_start = 20  # 最初のテキストのY座標
    y_step = 20   # 各テキスト行の間隔

    cv2.putText(combined_image, f"Steer (Pred): {steer_pred:.4f}", (text_x_start, y_start), font, 
                font_scale, color_pred, thickness)
    cv2.putText(combined_image, f"Steer (GT):   {steer_gt:.4f}", (text_x_start, y_start + y_step), font, 
                font_scale, color_gt, thickness)
    
    cv2.putText(combined_image, f"Speed (Pred): {speed_pred:.4f}", (text_x_start, y_start + y_step * 2 + 10), font, 
                font_scale, color_pred, thickness)
    cv2.putText(combined_image, f"Speed (GT):   {speed_gt:.4f}", (text_x_start, y_start + y_step * 3 + 10), font, 
                font_scale, color_gt, thickness)
    
    return combined_image

@hydra.main(config_path="config", config_name="evaluate", version_base="1.2")
def main(cfg: DictConfig) -> None:
    print("--- Evaluation Configuration ---")
    print(OmegaConf.to_yaml(cfg))
    print("--------------------------------")

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f"Using device: {device}")

    # 出力ディレクトリの設定 (Hydraは実行時にカレントディレクトリを変更します)
    output_dir = cfg.output_dir
    os.makedirs(output_dir, exist_ok=True)
    print(f"Saving results to: {os.path.abspath(output_dir)}")

    # データセットのパス設定
    base_path = hydra.utils.to_absolute_path(cfg.data_path)
    test_path = os.path.join(base_path, "test")

    if not os.path.exists(test_path):
        print(f"❌ Error: Test dataset not found at: {test_path}")
        return

    # =====================================================
    # Dataset: validation/test
    # =====================================================
    val_dataset = MultiSequenceDataset(
        base_dir=test_path,
        seq_len=cfg.dataset.sequence_length,
        transform=TestTransform(
            height=cfg.dataset.image_height,
            width=cfg.dataset.image_width,
            mode=cfg.dataset.transform_mode
        )
    )

    val_loader = DataLoader(
        val_dataset,
        batch_size=cfg.training.batch_size, # config (推奨: 1)
        shuffle=False,                     # 推論時はシャッフルしない
        num_workers=cfg.training.num_workers
    )
    print(f"✅ Loaded test dataset from: {test_path}")

    # =====================================================
    # モデルの読み込み
    # =====================================================
    model_size = cfg.model.size
    if model_size == 'tiny':
        model = TinyPilotNet(num_outputs=cfg.model.num_outputs).to(device)
    else:
        model = PilotNet(num_outputs=cfg.model.num_outputs).to(device)

    # チェックポイントのパス
    ckpt_path = hydra.utils.to_absolute_path(cfg.ckpt_path)
    if not os.path.exists(ckpt_path):
        print(f"❌ Error: Checkpoint file not found at: {ckpt_path}")
        return

    try:
        print(f"🔄 Loading weights from: {ckpt_path}")
        # CPUまたはGPUに重みをロード
        weights = torch.load(ckpt_path, map_location=device)
        
        if isinstance(weights, dict) and 'model_state_dict' in weights:
            # train.py が辞書で保存している場合 (将来的な拡張用)
            model.load_state_dict(weights['model_state_dict'])
        else:
            # train.py が state_dict を直接保存している場合
            model.load_state_dict(weights)
        print("✅ Weights loaded successfully.")
    except Exception as e:
        print(f"❌ Error loading weights: {e}")
        return

    # =====================================================
    # 推論ループ
    # =====================================================
    model.eval()
    frame_counter = 0

    with torch.no_grad():
        # データローダーからバッチ(B=1)を取得
        for batch in tqdm(val_loader, desc="Evaluating"):
            images = batch['image'].to(device) # Shape: [B, S, C, H, W]
            steers_gt = batch['steer']         # Shape: [B, S]
            speeds_gt = batch['speed']         # Shape: [B, S]

            b, s, c, h, w = images.shape
            
            # 学習時と同様に、(B*S) の形にフラット化してモデルに入力
            inputs = images.view(b * s, c, h, w)
            labels_gt = torch.stack([
                steers_gt.view(b * s),
                speeds_gt.view(b * s)
            ], dim=-1)
            
            # 推論実行
            outputs = model(inputs) # Shape: [B*S, 2]

            # 1枚ずつ処理して保存
            for i in range(b * s):
                image_tensor = inputs[i]   # [C, H, W]
                output = outputs[i]        # [2]
                label = labels_gt[i]       # [2]

                steer_pred = output[0].item()
                speed_pred = output[1].item()
                steer_gt = label[0].item()
                speed_gt = label[1].item()

                # テンソルをOpenCV画像(BGR)に変換
                img_bgr = tensor_to_cv_image(image_tensor)
                
                # 結果を描画
                img_result = draw_results_beside_image(img_bgr, steer_pred, steer_gt, speed_pred, speed_gt)

                # =========================================================
                # 【新しく追加】最終的な画像をリサイズして保存
                # =========================================================
                final_h, final_w, _ = img_result.shape
                resized_img_result = cv2.resize(
                    img_result, 
                    (int(final_w * OUTPUT_SCALE_FACTOR), int(final_h * OUTPUT_SCALE_FACTOR)), 
                    interpolation=cv2.INTER_LINEAR
                )

                # PNGとして保存
                save_path = os.path.join(output_dir, f"frame_{frame_counter:06d}.png")
                cv2.imwrite(save_path, resized_img_result) # <--- リサイズ後の画像を保存
                
                frame_counter += 1

    print(f"✅ Evaluation complete. {frame_counter} images saved to {os.path.abspath(output_dir)}")


if __name__ == '__main__':
    main()