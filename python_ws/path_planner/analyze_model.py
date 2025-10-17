import cv2
import numpy as np
import torch
from torch.utils.data import DataLoader
from scipy.spatial.transform import Rotation as R
import hydra
from omegaconf import DictConfig, OmegaConf
from pathlib import Path
from tqdm import tqdm
import time  

# 学習スクリプトで使用したモジュールを再利用
from src.data.dataset import TrajectoryDataset   
from src.data.transform import TestTransform
from src.model.trajformer import TrajFormer

# --- BEV描画関数 ---
def transform_global_to_local(ref_pose_7d: np.ndarray, target_poses_7d: np.ndarray) -> np.ndarray:
    ref_position = ref_pose_7d[:3]
    ref_quat = ref_pose_7d[3:]
    target_positions = target_poses_7d[:, :3]
    translated = target_positions - ref_position
    inv_rotation = R.from_quat(ref_quat).inv()
    local_coords = inv_rotation.apply(translated)
    return local_coords[:, :2]

def draw_trajectory_on_bev(future_path_local: np.ndarray, past_odom_global: np.ndarray | None = None,
                           canvas_size=(400,400), pixels_per_meter=50, max_speed_ms=2.0) -> np.ndarray:
    canvas = np.zeros((canvas_size[1], canvas_size[0], 3), dtype=np.uint8)
    robot_origin_u = canvas_size[0]//2
    robot_origin_v = canvas_size[1]-50

    robot_points = np.array([[robot_origin_u, robot_origin_v],
                             [robot_origin_u-10, robot_origin_v+15],
                             [robot_origin_u+10, robot_origin_v+15]])
    cv2.fillPoly(canvas, [robot_points], (255,255,255))

    # 過去軌跡
    if past_odom_global is not None and len(past_odom_global) > 1:
        current_pose_7d = past_odom_global[-1, :7]
        past_path_local = transform_global_to_local(current_pose_7d, past_odom_global[:, :7])
        for i, (x,y) in enumerate(past_path_local):
            u = int(robot_origin_u - y*pixels_per_meter)
            v = int(robot_origin_v - x*pixels_per_meter)
            color = (255,128,0)
            if i>0:
                prev_x, prev_y = past_path_local[i-1]
                prev_u = int(robot_origin_u - prev_y*pixels_per_meter)
                prev_v = int(robot_origin_v - prev_x*pixels_per_meter)
                cv2.line(canvas, (prev_u, prev_v), (u,v), color, 2)
            cv2.circle(canvas, (u,v), 3, color, -1)

    # 未来軌跡
    for i, (x,y,vx) in enumerate(future_path_local):
        u = int(robot_origin_u - y*pixels_per_meter)
        v = int(robot_origin_v - x*pixels_per_meter)
        speed_ratio = min(vx/max_speed_ms,1.0)
        color = (0,int(255*(1-speed_ratio)),int(255*speed_ratio))
        if i>0:
            prev_x, prev_y, _ = future_path_local[i-1]
            prev_u = int(robot_origin_u - prev_y*pixels_per_meter)
            prev_v = int(robot_origin_v - prev_x*pixels_per_meter)
            cv2.line(canvas, (prev_u,prev_v), (u,v), color, 2)
        cv2.circle(canvas,(u,v),3,color,-1)

    cv2.putText(canvas, "Future",(10,25),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,255,255),2)
    cv2.putText(canvas, "Past",(10,50),cv2.FONT_HERSHEY_SIMPLEX,0.7,(255,128,0),2)

    return canvas


# --- メイン (Hydra対応) ---
@hydra.main(config_path="config", config_name="analyze", version_base="1.2")
def main(cfg: DictConfig) -> None:
    print("--- Configuration ---")
    print(OmegaConf.to_yaml(cfg))
    print("---------------------")

    # --- デバイス設定 ---
    is_mps = torch.backends.mps.is_available() and cfg.device == 'cuda' # 'cuda'設定時にMPSも考慮
    if is_mps:
        device = torch.device('mps')
    elif torch.cuda.is_available() and cfg.device == 'cuda':
        device = torch.device('cuda')
    else:
        device = torch.device('cpu')
    print(f"Using device: {device}")


    # --- パス設定 (Hydraの to_absolute_path を使用) ---
    dataset_dir = Path(hydra.utils.to_absolute_path(cfg.dataset_dir))
    model_ckpt = Path(hydra.utils.to_absolute_path(cfg.ckpt_path))
    save_dir = Path(hydra.utils.to_absolute_path(cfg.output_dir))
    
    save_dir.mkdir(parents=True, exist_ok=True)

    # --- Dataset (Configから読み込み) ---
    transform = TestTransform(
        height=cfg.dataset.image_height, 
        width=cfg.dataset.image_width, 
        mode=cfg.dataset.transform_mode
    )
    dataset = TrajectoryDataset(root_dir=dataset_dir, transform=transform)
    # バッチサイズは推論時は 1 で固定
    loader = DataLoader(dataset, batch_size=1, shuffle=False, num_workers=cfg.training.num_workers)

    # --- モデル読み込み (Configから読み込み) ---
    model = TrajFormer(
        history_len=cfg.dataset.past_len,
        odom_features=cfg.model.odom_dim,
        future_len=cfg.dataset.future_len,
        image_embedding_dim=cfg.model.image_embedding_dim,
        motion_embedding_dim=cfg.model.motion_embedding_dim,
        transformer_d_model=cfg.model.d_model,
        transformer_nhead=cfg.model.transformer_nhead,
        transformer_num_layers=cfg.model.transformer_num_layers
    ).to(device)
    
    if not model_ckpt.exists():
        print(f"[ERROR] Checkpoint file not found: {model_ckpt}")
        return

    model.load_state_dict(torch.load(model_ckpt, map_location=device))
    model.eval()

    print(f"[INFO] Running inference and saving {len(dataset)} visualizations to {save_dir}")

    inference_times = [] # <-- 計測時間保存用リスト

    for idx, batch in enumerate(tqdm(loader, desc="Inference")):
        image = batch['image'].to(device)
        past_odoms = batch['past_odoms'].to(device)

        # --- 推論時間計測 開始 ---
        starter, ender = None, None
        start_time_cpu = None

        if device.type == 'cuda':
            starter, ender = torch.cuda.Event(enable_timing=True), torch.cuda.Event(enable_timing=True)
            starter.record()
        elif device.type == 'mps':
            starter, ender = torch.mps.Event(enable_timing=True), torch.mps.Event(enable_timing=True)
            starter.record()
        else: # CPU
            start_time_cpu = time.perf_counter()

        with torch.no_grad():
            pred_future_tensor = model(image, past_odoms)

        # --- 推論時間計測 終了 ---
        curr_time = 0.0
        if device.type == 'cuda':
            ender.record()
            torch.cuda.synchronize() # GPU処理の完了を待つ
            curr_time = starter.elapsed_time(ender) / 1000.0 # ミリ秒を秒に変換
        elif device.type == 'mps':
            ender.record()
            torch.mps.synchronize()
            curr_time = starter.elapsed_time(ender) / 1000.0 # ミリ秒を秒に変換
        else: # CPU
            curr_time = time.perf_counter() - start_time_cpu
        
        # 最初の1回はウォームアップとして除外する場合もあるが、ここでは含める
        inference_times.append(curr_time)
        # --- 計測ここまで ---

        pred_future = pred_future_tensor.cpu().numpy()[0]
        past_odom_np = past_odoms.cpu().numpy()[0]
        img_np = image.cpu().numpy()[0].transpose(1,2,0)  # C,H,W -> H,W,C
        
        # (cfg.dataset.transform_mode に応じて正規化を解除)
        if cfg.dataset.transform_mode:
             img_np = img_np * 0.5 + 0.5  # [-1,1] -> [0,1]

        img_np = (img_np * 255).clip(0,255).astype(np.uint8)

        bev_canvas = draw_trajectory_on_bev(pred_future, past_odom_np)

        # 高さを揃えて横に並べる
        img_h, img_w, _ = img_np.shape
        bev_h, bev_w, _ = bev_canvas.shape
        
        combined = bev_canvas
        if img_h > 0:
            scale_factor = bev_h / img_h
            resized_img = cv2.resize(img_np, (int(img_w*scale_factor), bev_h))
            combined = np.hstack([resized_img, bev_canvas])
            cv2.putText(combined, f"Sample {idx}", (10,30), cv2.FONT_HERSHEY_SIMPLEX,0.8,(255,255,255),2)
        else:
             cv2.putText(combined, f"Sample {idx}", (10,30), cv2.FONT_HERSHEY_SIMPLEX,0.8,(255,255,255),2)

        save_path = save_dir / f"pred_{idx:06d}.png"
        cv2.imwrite(str(save_path), combined)

    # --- ループ終了後、計測結果を表示 ---
    if inference_times:
        # 最初の数回をウォームアップとして除外することも可能
        # 例: inference_times = inference_times[5:] 
        
        avg_time = np.mean(inference_times)
        avg_fps = 1.0 / avg_time
        total_time = np.sum(inference_times)
        
        print("\n--- Inference Speed ---")
        print(f"Total samples:   {len(inference_times)}")
        print(f"Total time:      {total_time:.2f} s")
        print(f"Avg time/sample: {avg_time * 1000:.2f} ms") # ミリ秒で表示
        print(f"Avg FPS:         {avg_fps:.2f} Hz")
        print("-----------------------")

    print(f"[INFO] All visualizations saved to {save_dir}")


if __name__ == "__main__":
    main()