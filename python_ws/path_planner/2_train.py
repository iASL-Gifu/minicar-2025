import torch
import torch.nn as nn
from torch.utils.data import DataLoader
from tqdm import tqdm
import hydra
from omegaconf import DictConfig, OmegaConf
from pathlib import Path
import os
from torch.utils.tensorboard import SummaryWriter

from src.data.dataset import MultiSequenceDataset
from src.data.transform import TrainTransform, TestTransform
from src.model.trajcontrolnet import TrajControlFormer

# =========================================================
# 運動学的損失 
# =========================================================
def get_kinematic_loss(predicted_trajectory, accel_weight=1.0, jerk_weight=1.0):
    xy_coords = predicted_trajectory[..., :2]
    velocity = xy_coords[:, 1:, :] - xy_coords[:, :-1, :]
    acceleration = velocity[:, 1:, :] - velocity[:, :-1, :]
    jerk = acceleration[:, 1:, :] - acceleration[:, :-1, :]
    loss_accel = torch.mean(torch.norm(acceleration, p=2, dim=2))
    loss_jerk = torch.mean(torch.norm(jerk, p=2, dim=2))
    return (accel_weight * loss_accel) + (jerk_weight * loss_jerk)


# =========================================================
# 学習1エポック
# =========================================================
def train_one_epoch(model, dataloader, criterion_traj, criterion_cmd, optimizer, device, cfg, mode):
    model.train()
    total_loss, total_traj_loss, total_cmd_loss, total_smooth_loss = 0.0, 0.0, 0.0, 0.0

    if mode == 'all':
        model.train()
    elif mode == 'path_generation':
        model.trajformer.train()
    elif mode == 'path_follow':
        model.control_net.train()

    for batch in tqdm(dataloader, desc=f"Training ({mode})"):
        images = batch['image'].to(device)
        past_odoms = batch['past_odoms'].to(device)
        future_path = batch['future_path'].to(device)
        future_cmd = batch['future_cmd'].to(device)

        optimizer.zero_grad()
        if mode == 'all':
            predicted_traj, predicted_cmd = model(images, past_odoms)
            loss_traj = criterion_traj(predicted_traj, future_path)
            loss_cmd = criterion_cmd(predicted_cmd, future_cmd)
            loss_smooth = get_kinematic_loss(predicted_traj)
            loss = (cfg.training.loss_weights.traj * loss_traj) + \
                   (cfg.training.loss_weights.cmd * loss_cmd) + \
                   (cfg.training.loss_weights.smooth * loss_smooth)
        elif mode == 'path_generation':
            predicted_traj = model.trajformer(images, past_odoms)
            loss_traj = criterion_traj(predicted_traj, future_path)
            loss_smooth = get_kinematic_loss(predicted_traj)
            loss_cmd = torch.tensor(0.0, device=device)
            loss = (cfg.training.loss_weights.traj * loss_traj) + \
                   (cfg.training.loss_weights.smooth * loss_smooth)
        elif mode == 'path_follow':
            predicted_cmd = model.control_net(future_path)
            loss_cmd = criterion_cmd(predicted_cmd, future_cmd)
            loss_traj = torch.tensor(0.0, device=device)
            loss_smooth = torch.tensor(0.0, device=device)
            loss = cfg.training.loss_weights.cmd * loss_cmd

        loss.backward()
        optimizer.step()
        total_loss += loss.item()
        total_traj_loss += loss_traj.item()
        total_cmd_loss += loss_cmd.item()
        total_smooth_loss += loss_smooth.item()

    num_batches = len(dataloader)
    return {
        'total': total_loss / num_batches,
        'traj': total_traj_loss / num_batches,
        'cmd': total_cmd_loss / num_batches,
        'smooth': total_smooth_loss / num_batches
    }


# =========================================================
# 検証1エポック 
# =========================================================
def validate_one_epoch(model, dataloader, criterion_traj, criterion_cmd, device, cfg, mode):
    model.eval()
    total_loss, total_traj_loss, total_cmd_loss, total_smooth_loss = 0.0, 0.0, 0.0, 0.0
    with torch.no_grad():
        for batch in tqdm(dataloader, desc=f"Validation ({mode})"):
            images = batch['image'].to(device)
            past_odoms = batch['past_odoms'].to(device)
            future_path = batch['future_path'].to(device)
            future_cmd = batch['future_cmd'].to(device)

            if mode == 'all':
                predicted_traj, predicted_cmd = model(images, past_odoms)
                loss_traj = criterion_traj(predicted_traj, future_path)
                loss_cmd = criterion_cmd(predicted_cmd, future_cmd)
                loss_smooth = get_kinematic_loss(predicted_traj)
                loss = (cfg.training.loss_weights.traj * loss_traj) + \
                       (cfg.training.loss_weights.cmd * loss_cmd) + \
                       (cfg.training.loss_weights.smooth * loss_smooth)
            elif mode == 'path_generation':
                predicted_traj = model.trajformer(images, past_odoms)
                loss_traj = criterion_traj(predicted_traj, future_path)
                loss_smooth = get_kinematic_loss(predicted_traj)
                loss_cmd = torch.tensor(0.0, device=device)
                loss = (cfg.training.loss_weights.traj * loss_traj) + \
                       (cfg.training.loss_weights.smooth * loss_smooth)
            elif mode == 'path_follow':
                predicted_cmd = model.control_net(future_path)
                loss_cmd = criterion_cmd(predicted_cmd, future_cmd)
                loss_traj = torch.tensor(0.0, device=device)
                loss_smooth = torch.tensor(0.0, device=device)
                loss = cfg.training.loss_weights.cmd * loss_cmd

            total_loss += loss.item()
            total_traj_loss += loss_traj.item()
            total_cmd_loss += loss_cmd.item()
            total_smooth_loss += loss_smooth.item()

    num_batches = len(dataloader)
    return {
        'total': total_loss / num_batches,
        'traj': total_traj_loss / num_batches,
        'cmd': total_cmd_loss / num_batches,
        'smooth': total_smooth_loss / num_batches
    }


# =========================================================
# メイン
# =========================================================
@hydra.main(config_path="config", config_name="train", version_base="1.2")
def main(cfg: DictConfig) -> None:
    print("--- Configuration ---")
    print(OmegaConf.to_yaml(cfg))
    print("---------------------")

    # --- モードの取得と検証 ---
    mode = cfg.training.mode
    if mode not in ['all', 'path_generation', 'path_follow']:
        raise ValueError(f"Invalid training.mode: {mode}. Must be one of 'all', 'path_generation', 'path_follow'.")
    print(f"Running in mode: {mode}")

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    print(f"Using device: {device}")

    log_dir = hydra.utils.to_absolute_path(cfg.log_dir)
    ckpt_dir = hydra.utils.to_absolute_path(cfg.ckpt_dir)
    os.makedirs(log_dir, exist_ok=True)
    os.makedirs(ckpt_dir, exist_ok=True)
    writer = SummaryWriter(log_dir=log_dir)

    base_path = hydra.utils.to_absolute_path(cfg.data_path)
    train_path = os.path.join(base_path, "train")
    test_path = os.path.join(base_path, "test")

    # --- Dataset ---
    train_dataset = MultiSequenceDataset(
        base_dir=Path(train_path),
        transform=TrainTransform(
            height=cfg.dataset.image_height,
            width=cfg.dataset.image_width,
        )
    )
    train_loader = DataLoader(
        train_dataset, batch_size=cfg.training.batch_size, shuffle=True,
        num_workers=cfg.training.num_workers, pin_memory=True, drop_last=True
    )

    val_loader = None
    if os.path.exists(test_path):
        val_dataset = MultiSequenceDataset(
            base_dir=Path(test_path),
            transform=TestTransform(
                height=cfg.dataset.image_height,
                width=cfg.dataset.image_width,
            )
        )
        val_loader = DataLoader(
            val_dataset, batch_size=cfg.training.batch_size, shuffle=False,
            num_workers=cfg.training.num_workers, pin_memory=True
        )
        print(f"✅ Validation dataset found: {test_path}")
    else:
        print(f"⚠️ Validation dataset not found: {test_path}")

    # --- モデル定義 ---
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


    # =========================================================
    # ▼▼▼▼▼▼ 重み読み込みロジックの変更 ▼▼▼▼▼▼
    # =========================================================
    
    # 優先度1: 学習全体を再開する (resume_ckpt_path)
    resume_ckpt_path = cfg.get('resume_ckpt_path', None)
    
    # 優先度2: 個別モジュールの重みを読み込む (load_weights)
    load_weights_cfg = cfg.get('load_weights', None) 

    loaded_from_resume = False # resume_ckpt_path からロードしたかどうかのフラグ

    # 優先度1: resume_ckpt_path の処理
    if resume_ckpt_path:
        resume_ckpt_path_abs = hydra.utils.to_absolute_path(resume_ckpt_path)
        if os.path.exists(resume_ckpt_path_abs):
            print(f"🔄 [Resume] Loading weights from: {resume_ckpt_path_abs}")
            try:
                weights = torch.load(resume_ckpt_path_abs, map_location=device)
                
                if mode == 'all':
                    print(f"   Loading weights into 'model' (Mode: {mode})")
                    model.load_state_dict(weights)
                elif mode == 'path_generation':
                    print(f"   Loading weights into 'model.trajformer' (Mode: {mode})")
                    model.trajformer.load_state_dict(weights)
                elif mode == 'path_follow':
                    print(f"   Loading weights into 'model.control_net' (Mode: {mode})")
                    model.control_net.load_state_dict(weights)
                
                print("→ [Resume] Weights loaded successfully.")
                loaded_from_resume = True # ロード成功フラグ

            except RuntimeError as e:
                print(f"⚠️ [Resume] Failed to load weights (Key mismatch?): {e}")
                print(f"   Ensure checkpoint matches training mode ('{mode}').")
                print("→ Starting training from scratch (or checking individual weights).")
            except Exception as e:
                print(f"⚠️ [Resume] Failed to load weights (Other error): {e}")
                print("→ Starting training from scratch (or checking individual weights).")
        else:
            print(f"⚠️ [Resume] Checkpoint path specified but not found: {resume_ckpt_path_abs}")
            print("→ Starting training from scratch (or checking individual weights).")

    # 優先度2: resume_ckpt_path がない場合、load_weights の処理を試みる
    if not loaded_from_resume and load_weights_cfg:
        print("🔄 [Load Weights] Checking for individual module weights...")
        
        traj_path = load_weights_cfg.get('trajformer_path', None)
        ctrl_path = load_weights_cfg.get('controlnet_path', None)
        
        loaded_individual = False

        # 1. TrajFormer の重みをロード
        if traj_path:
            traj_path_abs = hydra.utils.to_absolute_path(traj_path)
            if os.path.exists(traj_path_abs):
                try:
                    weights = torch.load(traj_path_abs, map_location=device)
                    # strict=False : TrajFormerの重みのみをロードし、ControlNetのキーがなくてもエラーにしない
                    model.trajformer.load_state_dict(weights, strict=True) 
                    print(f"   ✅ Loaded 'model.trajformer' from: {traj_path_abs}")
                    loaded_individual = True
                except Exception as e:
                    print(f"   ⚠️ Failed to load 'model.trajformer' from {traj_path_abs}: {e}")
            else:
                print(f"   ⚠️ 'trajformer_path' specified but not found: {traj_path_abs}")
        
        # 2. ControlNet の重みをロード
        if ctrl_path:
            ctrl_path_abs = hydra.utils.to_absolute_path(ctrl_path)
            if os.path.exists(ctrl_path_abs):
                try:
                    weights = torch.load(ctrl_path_abs, map_location=device)
                    # strict=False : ControlNetの重みのみをロードし、TrajFormerのキーがなくてもエラーにしない
                    model.control_net.load_state_dict(weights, strict=True) 
                    print(f"   ✅ Loaded 'model.control_net' from: {ctrl_path_abs}")
                    loaded_individual = True
                except Exception as e:
                    print(f"   ⚠️ Failed to load 'model.control_net' from {ctrl_path_abs}: {e}")
            else:
                print(f"   ⚠️ 'controlnet_path' specified but not found: {ctrl_path_abs}")

        if not loaded_individual:
             print("→ [Load Weights] No valid individual weights found or specified.")
             print(f"🚀 Starting training from scratch (Mode: {mode}).")
        else:
            print(f"→ [Load Weights] Finished loading individual weights. Starting training (Mode: {mode}).")

    # 優先度3: どちらも指定されていない場合（スクラッチ）
    elif not loaded_from_resume:
        print(f"🚀 Starting training from scratch (Mode: {mode}).")
    
    # =========================================================
    # ▲▲▲▲▲▲ 重み読み込みロジックの変更 終了 ▲▲▲▲▲▲
    # =========================================================


    criterion_traj = nn.SmoothL1Loss()
    criterion_cmd = nn.SmoothL1Loss()
    
    # --- modeに応じてオプティマイザの対象パラメータ ---
    if mode == 'all':
        parameters = model.parameters()
        print("Optimizing: ALL parameters.")
    elif mode == 'path_generation':
        parameters = model.trajformer.parameters()
        for param in model.control_net.parameters():
            param.requires_grad = False
        print("Optimizing: ONLY TrajFormer parameters.")
    elif mode == 'path_follow':
        parameters = model.control_net.parameters()
        for param in model.trajformer.parameters():
            param.requires_grad = False
        print("Optimizing: ONLY ControlFormer parameters.")
    optimizer = torch.optim.Adam(parameters, lr=cfg.training.learning_rate)

    best_metric = float('inf')

    # --- 学習ループ ---
    for epoch in range(cfg.training.epochs):
        train_losses = train_one_epoch(
            model, train_loader, criterion_traj, criterion_cmd, optimizer, device, cfg, mode
        )
        writer.add_scalar('Loss/train_total', train_losses['total'], epoch)
        writer.add_scalar('Loss/train_traj', train_losses['traj'], epoch)
        writer.add_scalar('Loss/train_cmd', train_losses['cmd'], epoch)
        writer.add_scalar('Loss/train_smooth', train_losses['smooth'], epoch)
        print(f"Epoch {epoch+1}/{cfg.training.epochs} | Train Loss: {train_losses['total']:.4f} "
              f"(Traj: {train_losses['traj']:.4f}, Cmd: {train_losses['cmd']:.4f}, Smooth: {train_losses['smooth']:.4f})")

        if val_loader is not None:
            val_losses = validate_one_epoch(
                model, val_loader, criterion_traj, criterion_cmd, device, cfg, mode
            )
            writer.add_scalar('Loss/val_total', val_losses['total'], epoch)
            writer.add_scalar('Loss/val_traj', val_losses['traj'], epoch)
            writer.add_scalar('Loss/val_cmd', val_losses['cmd'], epoch)
            writer.add_scalar('Loss/val_smooth', val_losses['smooth'], epoch)
            print(f"→ Validation Loss: {val_losses['total']:.4f} "
                  f"(Traj: {val_losses['traj']:.4f}, Cmd: {val_losses['cmd']:.4f}, Smooth: {val_losses['smooth']:.4f})")
            current_metric = val_losses['total']
        else:
            current_metric = train_losses['total']

        # --- modeに応じて保存するファイル名と対象 ---
        if current_metric < best_metric:
            best_metric = current_metric
            if mode == 'all':
                state_dict = model.state_dict()
                save_name = 'best_model.pth'
            elif mode == 'path_generation':
                state_dict = model.trajformer.state_dict()
                save_name = 'best_trajformer.pth'
            elif mode == 'path_follow':
                state_dict = model.control_net.state_dict()
                save_name = 'best_controlformer.pth'
            torch.save(state_dict, os.path.join(ckpt_dir, save_name))
            print(f"✨ Improved best model ({save_name} metric={best_metric:.4f}) saved!")

        if mode == 'all':
            torch.save(model.state_dict(), os.path.join(ckpt_dir, 'last_model.pth'))
        elif mode == 'path_generation':
            torch.save(model.trajformer.state_dict(), os.path.join(ckpt_dir, 'last_trajformer.pth'))
        elif mode == 'path_follow':
            torch.save(model.control_net.state_dict(), os.path.join(ckpt_dir, 'last_controlformer.pth'))

    print("✅ Finished training.")
    writer.close()


if __name__ == '__main__':
    main()