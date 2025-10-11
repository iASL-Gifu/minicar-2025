import torch
from torch.utils.data import DataLoader, random_split, Subset
from tqdm import tqdm
import hydra
from omegaconf import DictConfig, OmegaConf
import os
import numpy as np
import matplotlib.pyplot as plt
from sklearn.neighbors import NearestNeighbors
from pathlib import Path
import time

from src.data.dataset import RecordingSequenceDataset
from src.data.transform import TestTransform
from src.model.pilotnet import PilotNet

@hydra.main(config_path="config", config_name="analyze", version_base="1.2")
def main(cfg: DictConfig) -> None:
    print("--- Analysis Configuration ---")
    print(OmegaConf.to_yaml(cfg))
    print("----------------------------")

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f"Using device: {device}")

    # --- 1. モデルの読み込み ---
    model_path = hydra.utils.to_absolute_path(cfg.model_path)
    if not os.path.exists(model_path):
        print(f"[ERROR] Model checkpoint not found at: {model_path}")
        return
        
    model = PilotNet(num_outputs=cfg.model.num_outputs).to(device)
    model.load_state_dict(torch.load(model_path, map_location=device))
    model.eval()
    print(f"Model loaded from {model_path}")

    # --- 2. テスト用データセットの準備 ---
    data_path = hydra.utils.to_absolute_path(cfg.data_path)
    full_dataset = RecordingSequenceDataset(root_dir=data_path, sequence_length=cfg.dataset.sequence_length)
    
    generator = torch.Generator().manual_seed(42)
    train_size = int(cfg.dataset.train_val_split_ratio * len(full_dataset))
    val_size = len(full_dataset) - train_size
    _, val_indices = random_split(range(len(full_dataset)), [train_size, val_size], generator=generator)
    
    val_dataset = Subset(full_dataset, val_indices)
    val_dataset.dataset.transform = TestTransform(height=cfg.dataset.image_height, width=cfg.dataset.image_width)

    test_loader = DataLoader(val_dataset, batch_size=cfg.analysis.batch_size, shuffle=False, num_workers=cfg.num_workers)
    
    # --- 3. 全テストデータに対して推論を実行 & 速度計測 ---
    all_predictions, all_odoms = [], []
    total_inference_time = 0.0
    
    # === 推論速度計測のためのウォームアップ ===
    if device.type == 'cuda':
        print("Warming up GPU...")
        dummy_input = torch.randn(1, 3, cfg.dataset.image_height, cfg.dataset.image_width, device=device)
        for _ in range(10):
            _ = model(dummy_input)

    print("Running inference and benchmarking speed...")
    with torch.no_grad():
        for batch in tqdm(test_loader, desc="Inference & Benchmark"):
            images = batch['image'].to(device)
            odoms = batch['odom']

            b, s, c, h, w = images.shape
            inputs = images.view(b * s, c, h, w)
            
            # === 時間計測開始 ===
            if device.type == 'cuda':
                start_event = torch.cuda.Event(enable_timing=True)
                end_event = torch.cuda.Event(enable_timing=True)
                start_event.record()

            else: # CPUの場合
                start_time = time.time()

            # --- 推論実行 ---
            predictions = model(inputs)
            
            # === 時間計測終了 ===
            if device.type == 'cuda':
                end_event.record()
                torch.cuda.synchronize() 
                elapsed_time = start_event.elapsed_time(end_event) / 1000.0
            else: # CPUの場合
                end_time = time.time()
                elapsed_time = end_time - start_time
            
            total_inference_time += elapsed_time

            all_predictions.append(predictions.cpu().numpy())
            all_odoms.append(odoms.view(b * s, -1).numpy())

    all_predictions = np.concatenate(all_predictions, axis=0)
    all_odoms = np.concatenate(all_odoms, axis=0)

    # --- 4. 推論速度の結果を計算・表示 ---
    num_images = len(all_odoms)
    avg_time_per_image = total_inference_time / num_images
    fps = 1.0 / avg_time_per_image

    print("\n--- Inference Speed Benchmark ---")
    print(f"Total images processed: {num_images}")
    print(f"Total inference time: {total_inference_time:.4f} seconds")
    print(f"Average time per image: {avg_time_per_image * 1000:.4f} ms")
    print(f"Frames Per Second (FPS): {fps:.2f} 🚀")
    print("---------------------------------\n")

    # --- 5. k-NNで近傍点の分散を計算 ---
    print("Analyzing prediction consistency using k-NN...")
    k = cfg.analysis.k_neighbors
    positions = all_odoms[:, :2] 

    nbrs = NearestNeighbors(n_neighbors=k, algorithm='ball_tree').fit(positions)
    distances, indices = nbrs.kneighbors(positions)

    variances = []
    for i in tqdm(range(len(positions)), desc="Calculating Variances"):
        neighbor_indices = indices[i]
        neighbor_steers = all_predictions[neighbor_indices, 0]
        variance = np.var(neighbor_steers)
        variances.append(variance)
    
    variances = np.array(variances)

    # --- 6. 結果の可視化と保存 ---
    print("Visualizing consistency results...")
    output_dir = Path(hydra.core.hydra_config.HydraConfig.get().runtime.output_dir)
    save_path = output_dir / "model_consistency_map.png"

    plt.style.use('seaborn-v0_8-darkgrid')
    fig, ax = plt.subplots(figsize=(12, 10))
    ax.plot(positions[:, 0], positions[:, 1], color='gray', alpha=0.3, zorder=1)
    scatter = ax.scatter(positions[:, 0], positions[:, 1], c=variances, cmap='jet', s=10, zorder=2, vmax=np.percentile(variances, 98))
    ax.set_title(f'Model Prediction Consistency (k={k})', fontsize=16)
    ax.set_xlabel('X Position [m]', fontsize=12)
    ax.set_ylabel('Y Position [m]', fontsize=12)
    ax.set_aspect('equal', adjustable='box')
    cbar = fig.colorbar(scatter, ax=ax)
    cbar.set_label('Steering Prediction Variance', fontsize=12)
    fig.tight_layout()
    plt.savefig(save_path, dpi=150)
    plt.close()

    print(f"✅ Analysis complete! Visualization saved to: {save_path}")

if __name__ == '__main__':
    main()