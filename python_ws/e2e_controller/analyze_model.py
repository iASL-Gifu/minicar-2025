import torch
import torch.multiprocessing
torch.multiprocessing.set_sharing_strategy('file_system')

from torch.utils.data import DataLoader
from tqdm import tqdm
import hydra
from omegaconf import DictConfig, OmegaConf
import numpy as np
import matplotlib.pyplot as plt
from sklearn.neighbors import NearestNeighbors
from pathlib import Path
import time
import os

from src.data.dataset import MultiSequenceDataset
from src.data.transform import TestTransform
from src.model.pilotnet import PilotNet


@hydra.main(config_path="config", config_name="analyze", version_base="1.2")
def main(cfg: DictConfig) -> None:
    print("--- Analysis Configuration ---")
    print(OmegaConf.to_yaml(cfg))
    print("----------------------------")

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f"Using device: {device}")

    # --- 1. モデル読み込み ---
    model_path = hydra.utils.to_absolute_path(cfg.model_path)
    if not os.path.exists(model_path):
        print(f"[ERROR] Model checkpoint not found at: {model_path}")
        return

    model = PilotNet(num_outputs=cfg.model.num_outputs).to(device)
    model.load_state_dict(torch.load(model_path, map_location=device))
    model.eval()
    print(f"✅ Model loaded from {model_path}")

    # --- 2. データセット全体を解析対象に ---
    data_path = hydra.utils.to_absolute_path(cfg.data_path)
    select_sequences = getattr(cfg.analysis, "select_sequences", None)
    dataset = MultiSequenceDataset(base_dir=data_path, seq_len=cfg.dataset.sequence_length, select_sequences=select_sequences)
    dataset.transform = TestTransform(height=cfg.dataset.image_height, width=cfg.dataset.image_width)

    loader = DataLoader(dataset, batch_size=cfg.analysis.batch_size, shuffle=False, num_workers=0)

    # --- 3. 推論＆速度計測 ---
    all_predictions, all_odoms = [], []
    total_inference_time = 0.0

    if device.type == 'cuda':
        print("Warming up GPU...")
        dummy_input = torch.randn(1, 3, cfg.dataset.image_height, cfg.dataset.image_width, device=device)
        for _ in range(10):
            _ = model(dummy_input)

    print("Running inference and benchmarking speed...")
    with torch.no_grad():
        for batch in tqdm(loader, desc="Inference & Benchmark"):
            images = batch['image'].to(device)
            odoms = batch['odom']

            b, s, c, h, w = images.shape
            inputs = images.view(b * s, c, h, w)

            # 時間計測
            if device.type == 'cuda':
                start_event = torch.cuda.Event(enable_timing=True)
                end_event = torch.cuda.Event(enable_timing=True)
                start_event.record()
            else:
                start_time = time.time()

            predictions = model(inputs)

            if device.type == 'cuda':
                end_event.record()
                torch.cuda.synchronize()
                elapsed_time = start_event.elapsed_time(end_event) / 1000.0
            else:
                elapsed_time = time.time() - start_time

            total_inference_time += elapsed_time

            all_predictions.append(predictions.cpu().numpy())
            all_odoms.append(odoms.view(b * s, -1).numpy())

    all_predictions = np.concatenate(all_predictions, axis=0)
    all_odoms = np.concatenate(all_odoms, axis=0)

    # --- 4. 推論速度計算 ---
    num_images = len(all_odoms)
    avg_time_per_image = total_inference_time / num_images
    fps = 1.0 / avg_time_per_image

    print("\n--- Inference Speed Benchmark ---")
    print(f"Total images processed: {num_images}")
    print(f"Total inference time: {total_inference_time:.4f} s")
    print(f"Average time per image: {avg_time_per_image * 1000:.4f} ms")
    print(f"Frames Per Second (FPS): {fps:.2f}")
    print("---------------------------------\n")

    # --- 5. 近傍点の分散解析 ---
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

    # --- 6. 可視化 ---
    print("Visualizing consistency results...")
    output_dir = Path(hydra.core.hydra_config.HydraConfig.get().runtime.output_dir)
    save_path = output_dir / "model_consistency_map.png"

    plt.style.use('seaborn-v0_8-darkgrid')
    fig, ax = plt.subplots(figsize=(12, 10))
    ax.plot(positions[:, 0], positions[:, 1], color='gray', alpha=0.3, zorder=1)
    scatter = ax.scatter(
        positions[:, 0],
        positions[:, 1],
        c=variances,
        cmap='jet',
        s=10,
        zorder=2,
        vmax=np.percentile(variances, 98)
    )
    ax.set_title(f'Model Prediction Consistency (k={k})\nTotal Points: {len(positions)}', fontsize=15)
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
