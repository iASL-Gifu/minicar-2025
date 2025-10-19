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
import os 

from src.data.dataset import MultiSequenceDataset   
from src.data.transform import TestTransform
from src.model.trajcontrolnet import TrajControlFormer

def transform_global_to_local(ref_pose_7d: np.ndarray, target_poses_7d: np.ndarray) -> np.ndarray:
    ref_position = ref_pose_7d[:3]
    ref_quat = ref_pose_7d[3:]
    target_positions = target_poses_7d[:, :3]
    translated = target_positions - ref_position
    inv_rotation = R.from_quat(ref_quat).inv()
    local_coords = inv_rotation.apply(translated)
    return local_coords[:, :2]

def draw_bev_pred_vs_gt(
    pred_future_local: np.ndarray, 
    gt_future_local: np.ndarray, 
    past_odom_global: np.ndarray | None = None,
    canvas_size=(400,400), pixels_per_meter=50, max_speed_ms=2.0
) -> np.ndarray:
    
    canvas = np.zeros((canvas_size[1], canvas_size[0], 3), dtype=np.uint8)
    robot_origin_u = canvas_size[0]//2
    robot_origin_v = canvas_size[1]-50

    robot_points = np.array([[robot_origin_u, robot_origin_v],
                             [robot_origin_u-10, robot_origin_v+15],
                             [robot_origin_u+10, robot_origin_v+15]])
    cv2.fillPoly(canvas, [robot_points], (255,255,255))

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

    cv2.putText(canvas, "Pred Future",(10,25),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,255,0),2)
    cv2.putText(canvas, "GT Future",(10,50),cv2.FONT_HERSHEY_SIMPLEX,0.7,(255,255,255),2)
    cv2.putText(canvas, "Past Odom",(10,75),cv2.FONT_HERSHEY_SIMPLEX,0.7,(255,128,0),2)

    return canvas

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

        gt_steer_ratio = np.clip(gt_cmds[i, 0] / max_steer_rad, -1.0, 1.0)
        gt_bar_height = int(gt_steer_ratio * (canvas_size[1] / 2 * 0.8))
        cv2.rectangle(canvas, (u_start, v_center), (u_end, v_center - gt_bar_height), (255, 255, 255), 1)

        pred_steer_ratio = np.clip(pred_cmds[i, 0] / max_steer_rad, -1.0, 1.0)
        pred_bar_height = int(pred_steer_ratio * (canvas_size[1] / 2 * 0.8))
        pred_color = (0, 200, 0) if pred_steer_ratio <= 0 else (0, 0, 200)
        cv2.rectangle(canvas, (u_start, v_center), (u_end, v_center - pred_bar_height), pred_color, -1)
        
    cv2.line(canvas, (0, v_center), (canvas_size[0] // 2, v_center), (255, 255, 255), 1)

    cv2.putText(canvas, "Speed (Pred/GT)", (canvas_size[0] // 2 + 10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    speed_base_v = canvas_size[1] - 20
    start_offset_speed = (canvas_size[0] // 2) + start_offset

    for i in range(num_cmds):
        u_start = start_offset_speed + (bar_width + bar_spacing) * i
        u_end = u_start + bar_width

        gt_speed_ratio = np.clip(gt_cmds[i, 1] / max_speed_ms, 0.0, 1.0)
        gt_bar_height = int(gt_speed_ratio * (canvas_size[1] * 0.8))
        cv2.rectangle(canvas, (u_start, speed_base_v), (u_end, speed_base_v - gt_bar_height), (255, 255, 255), 1)

        pred_speed_ratio = np.clip(pred_cmds[i, 1] / max_speed_ms, 0.0, 1.0)
        pred_bar_height = int(pred_speed_ratio * (canvas_size[1] * 0.8))
        cv2.rectangle(canvas, (u_start, speed_base_v), (u_end, speed_base_v - pred_bar_height), (0, 255, 0), -1)

    cv2.line(canvas, (canvas_size[0] // 2, speed_base_v), (canvas_size[0], speed_base_v), (255, 255, 255), 1)

    return canvas


@hydra.main(config_path="config", config_name="analyze", version_base="1.2")
def main(cfg: DictConfig) -> None:
    print("--- Configuration ---")
    print(OmegaConf.to_yaml(cfg))
    print("---------------------")

    is_mps = torch.backends.mps.is_available() and cfg.device == 'cuda'
    if is_mps: device = torch.device('mps')
    elif torch.cuda.is_available() and cfg.device == 'cuda': device = torch.device('cuda')
    else: device = torch.device('cpu')
    print(f"Using device: {device}")

    dataset_dir = Path(hydra.utils.to_absolute_path(cfg.data_path))
    model_ckpt = cfg.get('ckpt_path', None) 
    
    output_dir = hydra.core.hydra_config.HydraConfig.get().runtime.output_dir
    save_dir = Path(output_dir)

    transform = TestTransform(
        height=cfg.dataset.image_height, 
        width=cfg.dataset.image_width
    )
    select_sequences = getattr(cfg.analysis, "select_sequences", None)

    dataset = MultiSequenceDataset(base_dir=dataset_dir, transform=transform, sequence_indices=select_sequences)
    loader = DataLoader(dataset, batch_size=1, shuffle=False, num_workers=cfg.training.num_workers)

    model = TrajControlFormer(
        history_len=cfg.dataset.past_len,
        future_len=cfg.dataset.future_len,
        
        # --- YAML/configキーとクラス引数名を統一 ---
        odom_features=cfg.model.odom_features,
        image_embedding_dim=cfg.model.image_embedding_dim,
        motion_embedding_dim=cfg.model.motion_embedding_dim,
        transformer_d_model=cfg.model.transformer_d_model,
        transformer_nhead=cfg.model.transformer_nhead,
        transformer_num_layers=cfg.model.transformer_num_layers,

        control_motion_embedding_dim=cfg.model.control_motion_embedding_dim,
        control_d_model=cfg.model.control_d_model,
        control_nhead=cfg.model.control_nhead,
        control_num_layers=cfg.model.control_num_layers
    ).to(device)
    
    
    load_weights_cfg = cfg.get('load_weights', None)
    model_ckpt = cfg.get('ckpt_path', None) 

    loaded_weights = False
    draw_bev = False
    draw_cmd = False

    if load_weights_cfg:
        print("🔄 [Load Weights] Checking for individual module weights...")
        
        traj_path = load_weights_cfg.get('trajformer_path', None)
        ctrl_path = load_weights_cfg.get('controlnet_path', None)
        
        if traj_path:
            traj_path_abs = hydra.utils.to_absolute_path(traj_path)
            if os.path.exists(traj_path_abs):
                try:
                    weights = torch.load(traj_path_abs, map_location=device)
                    model.trajformer.load_state_dict(weights, strict=True)
                    print(f"   ✅ Loaded 'model.trajformer' from: {traj_path_abs}")
                    loaded_weights = True
                    draw_bev = True
                except Exception as e:
                    print(f"   ⚠️ Failed to load 'model.trajformer' from {traj_path_abs}: {e}")
            else:
                print(f"   ⚠️ 'trajformer_path' specified but not found: {traj_path_abs}")
        
        if ctrl_path:
            ctrl_path_abs = hydra.utils.to_absolute_path(ctrl_path)
            if os.path.exists(ctrl_path_abs):
                try:
                    weights = torch.load(ctrl_path_abs, map_location=device)
                    model.control_net.load_state_dict(weights, strict=True)
                    print(f"   ✅ Loaded 'model.control_net' from: {ctrl_path_abs}")
                    loaded_weights = True
                    draw_cmd = True
                except Exception as e:
                    print(f"   ⚠️ Failed to load 'model.control_net' from {ctrl_path_abs}: {e}")
            else:
                print(f"   ⚠️ 'controlnet_path' specified but not found: {ctrl_path_abs}")
        
        if loaded_weights:
            print(f"→ [Load Weights] Finished loading individual weights.")
        else:
            print(f"→ [Load Weights] No valid individual weights found.")

    if not loaded_weights and model_ckpt:
        model_ckpt_abs = hydra.utils.to_absolute_path(model_ckpt)
        if not os.path.exists(model_ckpt_abs):
            print(f"[ERROR] Checkpoint file not found: {model_ckpt_abs}")
            return
            
        print(f"🔄 [Single Ckpt] Loading weights from: {model_ckpt_abs}")
        print(f"   Assuming this is a full E2E model ('all').")
        state_dict = torch.load(model_ckpt_abs, map_location=device)
        
        try:
            model.load_state_dict(state_dict)
            print(f"[INFO] Loaded full model weights.")
            loaded_weights = True
            draw_bev = True
            draw_cmd = True
        except RuntimeError as e:
             print(f"⚠️ [Single Ckpt] Failed to load weights (Key mismatch?): {e}")

    if not loaded_weights:
        print("[WARN] No weights were loaded. Running inference with an uninitialized model.")
        draw_bev = True
        draw_cmd = True


    model.eval()

    print(f"[INFO] Running inference and saving {len(dataset)} visualizations to {save_dir}")
    print(f"   Visualizing BEV: {draw_bev}, Visualizing CMD: {draw_cmd}")
    inference_times = []

    for idx, batch in enumerate(tqdm(loader, desc="Inference")):
        image = batch['image'].to(device)
        past_odoms = batch['past_odoms'].to(device)
        gt_future_path = batch['future_path']
        gt_cmd = batch['future_cmd']

        start_time_cpu = None
        if device.type == 'cuda': starter, ender = torch.cuda.Event(enable_timing=True), torch.cuda.Event(enable_timing=True); starter.record()
        elif device.type == 'mps': starter, ender = torch.mps.Event(enable_timing=True), torch.mps.Event(enable_timing=True); starter.record()
        else: start_time_cpu = time.perf_counter()

        with torch.no_grad():
            pred_future_tensor, pred_cmd_tensor = model(image, past_odoms)

        curr_time = 0.0
        if device.type == 'cuda': ender.record(); torch.cuda.synchronize(); curr_time = starter.elapsed_time(ender) / 1000.0
        elif device.type == 'mps': ender.record(); torch.mps.synchronize(); curr_time = starter.elapsed_time(ender) / 1000.0
        else: curr_time = time.perf_counter() - start_time_cpu
        inference_times.append(curr_time)
        
        pred_future = pred_future_tensor.cpu().numpy()[0]
        pred_cmd = pred_cmd_tensor.cpu().numpy()[0]
        gt_future = gt_future_path.cpu().numpy()[0]
        gt_cmd = gt_cmd.cpu().numpy()[0]
        past_odom_np = past_odoms.cpu().numpy()[0]
        
        img_np = image.cpu().numpy()[0].transpose(1,2,0)
        img_np = img_np * 0.5 + 0.5
        img_np = (img_np * 255).clip(0,255).astype(np.uint8)
        img_np = cv2.cvtColor(img_np, cv2.COLOR_RGB2BGR)

        
        base_h = 400 
        base_w = 400 
        
        img_h, img_w, _ = img_np.shape
        scale_factor = base_h / img_h
        resized_img = cv2.resize(img_np, (int(img_w*scale_factor), base_h))
        
        panels_to_combine = [resized_img]

        if draw_bev:
            bev_canvas = draw_bev_pred_vs_gt(pred_future, gt_future, past_odom_np, (base_w, base_h))
            panels_to_combine.append(bev_canvas)

        if draw_cmd:
            cmd_canvas = draw_commands_panel(pred_cmd, gt_cmd, (base_w, base_h))
            panels_to_combine.append(cmd_canvas)

        combined = np.hstack(panels_to_combine)
        cv2.putText(combined, f"Sample {idx}", (10,30), cv2.FONT_HERSHEY_SIMPLEX,0.8,(255,255,255),2)

        save_path = save_dir / f"pred_{idx:06d}.png"
        cv2.imwrite(str(save_path), combined)

    if inference_times:
        if len(inference_times) > 1:
            inference_times_warmup = inference_times[1:]
        else:
            inference_times_warmup = inference_times
            
        avg_time = np.mean(inference_times_warmup)
        avg_fps = 1.0 / avg_time
        total_time = np.sum(inference_times)
        
        print("\n--- Inference Speed ---")
        print(f"Total samples:   {len(inference_times)}")
        if len(inference_times) > 1:
            print(f"(First sample (warmup): {inference_times[0]*1000:.2f} ms)")
            print(f"Total time (incl. warmup): {total_time:.2f} s")
            print(f"Avg time/sample (excl. warmup): {avg_time * 1000:.2f} ms")
            print(f"Avg FPS (excl. warmup):         {avg_fps:.2f} Hz")
        else:
            print(f"Total time:      {total_time:.2f} s")
            print(f"Avg time/sample: {avg_time * 1000:.2f} ms")
            print(f"Avg FPS:         {avg_fps:.2f} Hz")
        print("-----------------------")

    print(f"[INFO] All visualizations saved to {save_dir}")


if __name__ == "__main__":
    main()