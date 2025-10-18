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

# --- TrajControlFormer をインポート ---
from src.data.dataset import TrajectoryDataset   
from src.data.transform import TestTransform
from src.model.trajcontrolnet import TrajControlFormer # TrajFormer から変更

# --- BEV描画関数---
def transform_global_to_local(ref_pose_7d: np.ndarray, target_poses_7d: np.ndarray) -> np.ndarray:
    ref_position = ref_pose_7d[:3]
    ref_quat = ref_pose_7d[3:]
    target_positions = target_poses_7d[:, :3]
    translated = target_positions - ref_position
    inv_rotation = R.from_quat(ref_quat).inv()
    local_coords = inv_rotation.apply(translated)
    return local_coords[:, :2]

# --- BEV描画関数 (予測 vs 正解) ---
def draw_bev_pred_vs_gt(
    pred_future_local: np.ndarray, 
    gt_future_local: np.ndarray, 
    past_odom_global: np.ndarray | None = None,
    canvas_size=(400,400), pixels_per_meter=50, max_speed_ms=2.0
) -> np.ndarray:
    
    canvas = np.zeros((canvas_size[1], canvas_size[0], 3), dtype=np.uint8)
    robot_origin_u = canvas_size[0]//2
    robot_origin_v = canvas_size[1]-50

    # ロボット描画
    robot_points = np.array([[robot_origin_u, robot_origin_v],
                             [robot_origin_u-10, robot_origin_v+15],
                             [robot_origin_u+10, robot_origin_v+15]])
    cv2.fillPoly(canvas, [robot_points], (255,255,255))

    # 過去軌跡 (青)
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

    # 1. 未来軌跡 (Ground Truth) (白)
    gt_color = (255, 255, 255)
    for i, (x,y,vx) in enumerate(gt_future_local):
        u = int(robot_origin_u - y*pixels_per_meter)
        v = int(robot_origin_v - x*pixels_per_meter)
        if i>0:
            prev_x, prev_y, _ = gt_future_local[i-1]
            prev_u = int(robot_origin_u - prev_y*pixels_per_meter)
            prev_v = int(robot_origin_v - prev_x*pixels_per_meter)
            cv2.line(canvas, (prev_u,prev_v), (u,v), gt_color, 2)
        cv2.circle(canvas,(u,v), 3, gt_color, -1)

    # 2. 未来軌跡 (Prediction) (緑->赤)
    for i, (x,y,vx) in enumerate(pred_future_local):
        u = int(robot_origin_u - y*pixels_per_meter)
        v = int(robot_origin_v - x*pixels_per_meter)
        speed_ratio = min(vx/max_speed_ms,1.0)
        color = (0,int(255*(1-speed_ratio)),int(255*speed_ratio))
        if i>0:
            prev_x, prev_y, _ = pred_future_local[i-1]
            prev_u = int(robot_origin_u - prev_y*pixels_per_meter)
            prev_v = int(robot_origin_v - prev_x*pixels_per_meter)
            cv2.line(canvas, (prev_u,prev_v), (u,v), color, 2)
        cv2.circle(canvas,(u,v), 3, color, -1)

    # 凡例
    cv2.putText(canvas, "Pred Future",(10,25),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,255,255),2) # 黄
    cv2.putText(canvas, "GT Future",(10,50),cv2.FONT_HERSHEY_SIMPLEX,0.7,(255,255,255),2) # 白
    cv2.putText(canvas, "Past Odom",(10,75),cv2.FONT_HERSHEY_SIMPLEX,0.7,(255,128,0),2) # 青

    return canvas

# --- コマンド描画関数 (予測 vs 正解) ---
def draw_commands_panel(
    pred_cmds: np.ndarray, 
    gt_cmds: np.ndarray,
    canvas_size: tuple[int, int] = (400, 400),
    max_steer_rad: float = 0.6,
    max_speed_ms: float = 2.0
) -> np.ndarray:
    
    canvas = np.zeros((canvas_size[1], canvas_size[0], 3), dtype=np.uint8)
    num_cmds = len(pred_cmds)
    if num_cmds == 0:
        return canvas

    # --- ステアリング (左半分) ---
    cv2.putText(canvas, "Steer (Pred/GT)", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    bar_width_total = (canvas_size[0] // 2) * 0.8
    bar_width = int(bar_width_total / num_cmds)
    bar_spacing = 0
    if num_cmds > 1:
        bar_spacing = int(((canvas_size[0] // 2) * 0.2) / (num_cmds - 1))
    
    start_offset = (canvas_size[0] // 2 - (bar_width * num_cmds + bar_spacing * (num_cmds - 1))) // 2

    for i in range(num_cmds):
        u_start = start_offset + (bar_width + bar_spacing) * i
        u_end = u_start + bar_width
        v_center = canvas_size[1] // 2

        # GT (枠線: 白)
        gt_steer_ratio = np.clip(gt_cmds[i, 0] / max_steer_rad, -1.0, 1.0)
        gt_bar_height = int(gt_steer_ratio * (canvas_size[1] / 2 * 0.8))
        cv2.rectangle(canvas, (u_start, v_center), (u_end, v_center - gt_bar_height), (255, 255, 255), 1)

        # Pred (塗りつぶし: 緑/赤)
        pred_steer_ratio = np.clip(pred_cmds[i, 0] / max_steer_rad, -1.0, 1.0)
        pred_bar_height = int(pred_steer_ratio * (canvas_size[1] / 2 * 0.8))
        pred_color = (0, 200, 0) if pred_steer_ratio <= 0 else (0, 0, 200) # 緑:左, 赤:右
        cv2.rectangle(canvas, (u_start, v_center), (u_end, v_center - pred_bar_height), pred_color, -1)
        
    cv2.line(canvas, (0, v_center), (canvas_size[0] // 2, v_center), (255, 255, 255), 1)

    # --- 速度 (右半分) ---
    cv2.putText(canvas, "Speed (Pred/GT)", (canvas_size[0] // 2 + 10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    speed_base_v = canvas_size[1] - 20
    start_offset_speed = (canvas_size[0] // 2) + start_offset

    for i in range(num_cmds):
        u_start = start_offset_speed + (bar_width + bar_spacing) * i
        u_end = u_start + bar_width

        # GT (枠線: 白)
        gt_speed_ratio = np.clip(gt_cmds[i, 1] / max_speed_ms, 0.0, 1.0)
        gt_bar_height = int(gt_speed_ratio * (canvas_size[1] * 0.8))
        cv2.rectangle(canvas, (u_start, speed_base_v), (u_end, speed_base_v - gt_bar_height), (255, 255, 255), 1)

        # Pred (塗りつぶし: 緑)
        pred_speed_ratio = np.clip(pred_cmds[i, 1] / max_speed_ms, 0.0, 1.0)
        pred_bar_height = int(pred_speed_ratio * (canvas_size[1] * 0.8))
        cv2.rectangle(canvas, (u_start, speed_base_v), (u_end, speed_base_v - pred_bar_height), (0, 255, 0), -1)

    cv2.line(canvas, (canvas_size[0] // 2, speed_base_v), (canvas_size[0], speed_base_v), (255, 255, 255), 1)

    return canvas


# --- メイン (Hydra対応) ---
@hydra.main(config_path="config", config_name="analyze", version_base="1.2")
def main(cfg: DictConfig) -> None:
    print("--- Configuration ---")
    print(OmegaConf.to_yaml(cfg))
    print("---------------------")

    # デバイス設定
    is_mps = torch.backends.mps.is_available() and cfg.device == 'cuda'
    if is_mps: device = torch.device('mps')
    elif torch.cuda.is_available() and cfg.device == 'cuda': device = torch.device('cuda')
    else: device = torch.device('cpu')
    print(f"Using device: {device}")

    # パス設定
    dataset_dir = Path(hydra.utils.to_absolute_path(cfg.data_path))
    model_ckpt = Path(hydra.utils.to_absolute_path(cfg.ckpt_path))

    output_dir = hydra.core.hydra_config.HydraConfig.get().runtime.output_dir
    save_dir = Path(output_dir)

    # --- Dataset ---
    # configからtransform_modeを削除 (TestTransformはmode引数を持たないため)
    transform = TestTransform(
        height=cfg.dataset.image_height, 
        width=cfg.dataset.image_width
    )
    dataset = TrajectoryDataset(root_dir=dataset_dir, transform=transform)
    loader = DataLoader(dataset, batch_size=1, shuffle=False, num_workers=cfg.training.num_workers)

    # --- モデル読み込み (TrajControlFormer) ---
    model = TrajControlFormer(
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

    # --- モデル読み込みロジック (load_part に応じる) ---
    state_dict = torch.load(model_ckpt, map_location=device)
    if cfg.load_part == 'all':
        model.load_state_dict(state_dict)
        print(f"[INFO] Loaded full model weights from: {model_ckpt}")
    elif cfg.load_part == 'trajformer_only':
        model.trajformer.load_state_dict(state_dict)
        print(f"[INFO] Loaded TrajFormer weights into model.trajformer from: {model_ckpt}")
    elif cfg.load_part == 'control_only':
        model.control_net.load_state_dict(state_dict)
        print(f"[INFO] Loaded ControlFormer weights into model.control_net from: {model_ckpt}")
    else:
        print(f"[WARN] Unknown cfg.load_part: {cfg.load_part}. Attempting to load full model.")
        model.load_state_dict(state_dict)
        
    model.eval()

    print(f"[INFO] Running inference and saving {len(dataset)} visualizations to {save_dir}")
    inference_times = []

    for idx, batch in enumerate(tqdm(loader, desc="Inference")):
        image = batch['image'].to(device)
        past_odoms = batch['past_odoms'].to(device)
        # Ground Truth を取得
        gt_future_path = batch['future_path']
        gt_cmd = batch['future_cmd']

        # --- 推論時間計測 ---
        start_time_cpu = None
        if device.type == 'cuda': starter, ender = torch.cuda.Event(enable_timing=True), torch.cuda.Event(enable_timing=True); starter.record()
        elif device.type == 'mps': starter, ender = torch.mps.Event(enable_timing=True), torch.mps.Event(enable_timing=True); starter.record()
        else: start_time_cpu = time.perf_counter()

        with torch.no_grad():
            # 2つの出力を取得
            pred_future_tensor, pred_cmd_tensor = model(image, past_odoms)

        # --- 推論時間計測 終了 ---
        curr_time = 0.0
        if device.type == 'cuda': ender.record(); torch.cuda.synchronize(); curr_time = starter.elapsed_time(ender) / 1000.0
        elif device.type == 'mps': ender.record(); torch.mps.synchronize(); curr_time = starter.elapsed_time(ender) / 1000.0
        else: curr_time = time.perf_counter() - start_time_cpu
        inference_times.append(curr_time)
        
        # --- データをNumpyに変換 ---
        pred_future = pred_future_tensor.cpu().numpy()[0]
        pred_cmd = pred_cmd_tensor.cpu().numpy()[0]
        gt_future = gt_future_path.cpu().numpy()[0]
        gt_cmd = gt_cmd.cpu().numpy()[0]
        past_odom_np = past_odoms.cpu().numpy()[0]
        
        # 画像をH,W,Cに戻し、正規化を解除
        img_np = image.cpu().numpy()[0].transpose(1,2,0)
        img_np = img_np * 0.5 + 0.5  # [-1,1] -> [0,1]
        img_np = (img_np * 255).clip(0,255).astype(np.uint8)
        img_np = cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR) # OpenCV用にBGRに戻す

        # --- 描画と結合 (cfg.load_part に応じて) ---
        
        # 1. BEV (trajformer_only または all の場合)
        bev_canvas = None
        if cfg.load_part == 'trajformer_only' or cfg.load_part == 'all':
            bev_canvas = draw_bev_pred_vs_gt(pred_future, gt_future, past_odom_np)

        # 2. コマンド (control_only または all の場合)
        cmd_canvas = None
        if cfg.load_part == 'control_only' or cfg.load_part == 'all':
            cmd_canvas = draw_commands_panel(pred_cmd, gt_cmd)

        # 3. 基準となる高さを決定 (BEV優先、なければCMD基準)
        base_h = 400 # デフォルト
        base_w = 400 # デフォルト
        if bev_canvas is not None:
            base_h, base_w, _ = bev_canvas.shape
        elif cmd_canvas is not None:
            base_h, base_w, _ = cmd_canvas.shape
        
        # 4. 画像を基準の高さにリサイズ
        img_h, img_w, _ = img_np.shape
        scale_factor = base_h / img_h
        resized_img = cv2.resize(img_np, (int(img_w*scale_factor), base_h))
        
        panels_to_combine = [resized_img]

        # 5. パネルをリストに追加
        if bev_canvas is not None:
            panels_to_combine.append(bev_canvas)

        if cmd_canvas is not None:
            # もしBEVも存在する場合、CMDのサイズをBEVに合わせる (元コードのロジック踏襲)
            if bev_canvas is not None and (cmd_canvas.shape[0] != base_h or cmd_canvas.shape[1] != base_w):
                resized_cmd = cv2.resize(cmd_canvas, (base_w, base_h))
                panels_to_combine.append(resized_cmd)
            else:
                 # BEVがない場合、またはサイズが同じ場合はそのまま追加
                 panels_to_combine.append(cmd_canvas)


        # 6. 存在するパネルだけを結合
        combined = np.hstack(panels_to_combine)
        cv2.putText(combined, f"Sample {idx}", (10,30), cv2.FONT_HERSHEY_SIMPLEX,0.8,(255,255,255),2)

        save_path = save_dir / f"pred_{idx:06d}.png"
        cv2.imwrite(str(save_path), combined)

    # --- 計測結果を表示 ---
    if inference_times:
        avg_time = np.mean(inference_times)
        avg_fps = 1.0 / avg_time
        total_time = np.sum(inference_times)
        
        print("\n--- Inference Speed ---")
        print(f"Total samples:   {len(inference_times)}")
        print(f"Total time:      {total_time:.2f} s")
        print(f"Avg time/sample: {avg_time * 1000:.2f} ms")
        print(f"Avg FPS:         {avg_fps:.2f} Hz")
        print("-----------------------")

    print(f"[INFO] All visualizations saved to {save_dir}")


if __name__ == "__main__":
    main()