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

def get_kinematic_loss(predicted_trajectory, accel_weight=1.0, jerk_weight=1.0):
    xy_coords = predicted_trajectory[..., :2]
    velocity = xy_coords[:, 1:, :] - xy_coords[:, :-1, :]
    acceleration = velocity[:, 1:, :] - velocity[:, :-1, :]
    jerk = acceleration[:, 1:, :] - acceleration[:, :-1, :]
    loss_accel = torch.mean(torch.norm(acceleration, p=2, dim=2))
    loss_jerk = torch.mean(torch.norm(jerk, p=2, dim=2))
    return (accel_weight * loss_accel) + (jerk_weight * loss_jerk)

def get_control_smoothness_loss(predicted_cmd, rate_weight=1.0, accel_weight=0.5):
    """
    制御コマンド (steer, accel) の時間変化率を罰則化する
    """
    control_rate = predicted_cmd[:, 1:, :] - predicted_cmd[:, :-1, :]
    control_accel = control_rate[:, 1:, :] - control_rate[:, :-1, :]
    loss_rate = torch.mean(torch.norm(control_rate, p=2, dim=2))
    loss_accel = torch.mean(torch.norm(control_accel, p=2, dim=2))

    return (rate_weight * loss_rate) + (accel_weight * loss_accel)


def train_one_epoch(model, dataloader, criterion_traj, criterion_cmd, optimizer, device, cfg, mode):
    # (★修正) total_smooth_cmd_loss を追加
    total_loss, total_traj_loss, total_cmd_loss, total_smooth_loss, total_smooth_cmd_loss = 0.0, 0.0, 0.0, 0.0, 0.0

    if mode == 'all':
        model.train()
    elif mode == 'path_generation':
        model.trajformer.train()
    elif mode == 'path_follow':
        model.control_net.train()
        model.trajformer.eval() 

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
            loss_smooth_cmd = get_control_smoothness_loss(predicted_cmd)
            
            loss = (cfg.training.loss_weights.traj * loss_traj) + \
                   (cfg.training.loss_weights.cmd * loss_cmd) + \
                   (cfg.training.loss_weights.smooth * loss_smooth) + \
                   (cfg.training.loss_weights.smooth_cmd * loss_smooth_cmd)
        
        elif mode == 'path_generation':
            predicted_traj = model.trajformer(images, past_odoms)
            loss_traj = criterion_traj(predicted_traj, future_path)
            loss_smooth = get_kinematic_loss(predicted_traj)
            loss_cmd = torch.tensor(0.0, device=device)
            loss_smooth_cmd = torch.tensor(0.0, device=device) 
            loss = (cfg.training.loss_weights.traj * loss_traj) + \
                   (cfg.training.loss_weights.smooth * loss_smooth)

        elif mode == 'path_follow':
            use_gt_prob = cfg.training.get('scheduled_sampling_prob', 1.0)
            
            input_path_for_control = None

            if torch.rand(1).item() < use_gt_prob:
                input_path_for_control = future_path
                loss_traj = torch.tensor(0.0, device=device)
                loss_smooth = torch.tensor(0.0, device=device)
            
            else:
                with torch.no_grad():
                    input_path_for_control = model.trajformer(images, past_odoms)
                
                loss_traj = criterion_traj(input_path_for_control, future_path)
                loss_smooth = get_kinematic_loss(input_path_for_control)

            predicted_cmd = model.control_net(input_path_for_control, past_odoms)
            loss_cmd = criterion_cmd(predicted_cmd, future_cmd)
            loss_smooth_cmd = get_control_smoothness_loss(predicted_cmd)
            
            loss = (cfg.training.loss_weights.cmd * loss_cmd) + \
                   (cfg.training.loss_weights.smooth_cmd * loss_smooth_cmd)

        loss.backward()
        optimizer.step()
        total_loss += loss.item()
        total_traj_loss += loss_traj.item()
        total_cmd_loss += loss_cmd.item()
        total_smooth_loss += loss_smooth.item()
        total_smooth_cmd_loss += loss_smooth_cmd.item()

    num_batches = len(dataloader)
    return {
        'total': total_loss / num_batches,
        'traj': total_traj_loss / num_batches,
        'cmd': total_cmd_loss / num_batches,
        'smooth': total_smooth_loss / num_batches,
        'smooth_cmd': total_smooth_cmd_loss / num_batches
    }


def validate_one_epoch(model, dataloader, criterion_traj, criterion_cmd, device, cfg, mode):
    model.eval()
    total_loss, total_traj_loss, total_cmd_loss, total_smooth_loss, total_smooth_cmd_loss = 0.0, 0.0, 0.0, 0.0, 0.0
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
                loss_smooth_cmd = get_control_smoothness_loss(predicted_cmd) 
                loss = (cfg.training.loss_weights.traj * loss_traj) + \
                       (cfg.training.loss_weights.cmd * loss_cmd) + \
                       (cfg.training.loss_weights.smooth * loss_smooth) + \
                       (cfg.training.loss_weights.smooth_cmd * loss_smooth_cmd) 
            
            elif mode == 'path_generation':
                predicted_traj = model.trajformer(images, past_odoms)
                loss_traj = criterion_traj(predicted_traj, future_path)
                loss_smooth = get_kinematic_loss(predicted_traj)
                loss_cmd = torch.tensor(0.0, device=device)
                loss_smooth_cmd = torch.tensor(0.0, device=device)
                loss = (cfg.training.loss_weights.traj * loss_traj) + \
                       (cfg.training.loss_weights.smooth * loss_smooth)

            elif mode == 'path_follow':
                predicted_traj = model.trajformer(images, past_odoms)
                predicted_cmd = model.control_net(predicted_traj, past_odoms)

                loss_cmd = criterion_cmd(predicted_cmd, future_cmd)
                loss_traj = criterion_traj(predicted_traj, future_path)
                loss_smooth = get_kinematic_loss(predicted_traj)
                loss_smooth_cmd = get_control_smoothness_loss(predicted_cmd)
                
                loss = (cfg.training.loss_weights.traj * loss_traj) + \
                       (cfg.training.loss_weights.cmd * loss_cmd) + \
                       (cfg.training.loss_weights.smooth * loss_smooth) + \
                       (cfg.training.loss_weights.smooth_cmd * loss_smooth_cmd)
                
            total_loss += loss.item()
            total_traj_loss += loss_traj.item()
            total_cmd_loss += loss_cmd.item()
            total_smooth_loss += loss_smooth.item()
            total_smooth_cmd_loss += loss_smooth_cmd.item()

    num_batches = len(dataloader)
    return {
        'total': total_loss / num_batches,
        'traj': total_traj_loss / num_batches,
        'cmd': total_cmd_loss / num_batches,
        'smooth': total_smooth_loss / num_batches,
        'smooth_cmd': total_smooth_cmd_loss / num_batches
    }


@hydra.main(config_path="config", config_name="train", version_base="1.2")
def main(cfg: DictConfig) -> None:
    print("--- Configuration ---")
    print(OmegaConf.to_yaml(cfg))
    print("---------------------")

    mode = cfg.training.mode
    if mode not in ['all', 'path_generation', 'path_follow']:
        raise ValueError(f"Invalid training.mode: {mode}. Must be one of 'all', 'path_generation', 'path_follow'.")
    print(f"Running in mode: {mode}")
    
    if mode == 'path_follow':
        prob = cfg.training.get('scheduled_sampling_prob', 1.0)
        print(f"   Scheduled Sampling enabled (GT path prob = {prob * 100:.1f}%)")

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

    model = TrajControlFormer(
        history_len=cfg.dataset.past_len,
        odom_features=cfg.model.odom_features,
        future_len=cfg.dataset.future_len,
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
    
    loaded_traj = False

    if load_weights_cfg:
        print("🔄 [Load Weights] Checking for individual module weights...")
        
        traj_path = load_weights_cfg.get('trajformer_path', None)
        ctrl_path = load_weights_cfg.get('controlnet_path', None)
        
        loaded_individual = False

        if traj_path:
            traj_path_abs = hydra.utils.to_absolute_path(traj_path)
            if os.path.exists(traj_path_abs):
                try:
                    weights = torch.load(traj_path_abs, map_location=device)
                    model.trajformer.load_state_dict(weights, strict=True) 
                    print(f"   ✅ Loaded 'model.trajformer' from: {traj_path_abs}")
                    loaded_individual = True
                    loaded_traj = True
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
                    loaded_individual = True
                except Exception as e:
                    print(f"   ⚠️ Failed to load 'model.control_net' from {ctrl_path_abs}: {e}")
            else:
                print(f"   ⚠️ 'controlnet_path' specified but not found: {ctrl_path_abs}")
        
        if loaded_individual:
            print("→ [Load Weights] Finished loading individual weights.")
        else:
            print("→ [Load Weights] No valid individual weights found or specified.")


    resume_ckpt_path = cfg.get('resume_ckpt_path', None)
    
    if resume_ckpt_path:
        resume_ckpt_path_abs = hydra.utils.to_absolute_path(resume_ckpt_path)
        if os.path.exists(resume_ckpt_path_abs):
            print(f"🔄 [Resume] Loading weights from: {resume_ckpt_path_abs}")
            print(f"   This will OVERWRITE any weights loaded from 'load_weights' for the current mode ('{mode}').")
            try:
                weights = torch.load(resume_ckpt_path_abs, map_location=device)
                
                if mode == 'all':
                    print(f"   Loading weights into 'model' (Mode: {mode})")
                    model.load_state_dict(weights)
                    loaded_traj = True
                elif mode == 'path_generation':
                    print(f"   Loading weights into 'model.trajformer' (Mode: {mode})")
                    model.trajformer.load_state_dict(weights)
                    loaded_traj = True
                elif mode == 'path_follow':
                    print(f"   Loading weights into 'model.control_net' (Mode: {mode})")
                    model.control_net.load_state_dict(weights)
                
                print("→ [Resume] Weights loaded successfully.")

            except RuntimeError as e:
                print(f"⚠️ [Resume] Failed to load weights (Key mismatch?): {e}")
                print(f"   Ensure checkpoint matches training mode ('{mode}').")
            except Exception as e:
                print(f"⚠️ [Resume] Failed to load weights (Other error): {e}")
        else:
            print(f"⚠️ [Resume] Checkpoint path specified but not found: {resume_ckpt_path_abs}")
    
    else:
        if not load_weights_cfg or not loaded_individual:
             print(f"🚀 Starting training from scratch (Mode: {mode}).")
        else:
             print(f"🚀 Starting training from pre-loaded weights (Mode: {mode}).")

    use_gt_prob = cfg.training.get('scheduled_sampling_prob', 1.0)
    if mode == 'path_follow' and use_gt_prob < 1.0:
        if not loaded_traj:
             print(f"")
             print(f"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! WARNING !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
             print(f" 'path_follow' mode is running with Scheduled Sampling (prob < 1.0),")
             print(f" but 'load_weights.trajformer_path' was NOT specified or found.")
             print(f" → ControlNet will be trained using paths from an UNTRAINED TrajFormer.")
             print(f" → This is likely NOT what you want.")
             print(f" → Please specify a pre-trained 'trajformer_path' in 'load_weights'.")
             print(f"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
             print(f"")

    criterion_traj = nn.SmoothL1Loss()
    criterion_cmd = nn.SmoothL1Loss()
    
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

    for epoch in range(cfg.training.epochs):
        train_losses = train_one_epoch(
            model, train_loader, criterion_traj, criterion_cmd, optimizer, device, cfg, mode
        )
        writer.add_scalar('Loss/train_total', train_losses['total'], epoch)
        writer.add_scalar('Loss/train_traj', train_losses['traj'], epoch)
        writer.add_scalar('Loss/train_cmd', train_losses['cmd'], epoch)
        writer.add_scalar('Loss/train_smooth', train_losses['smooth'], epoch)
        writer.add_scalar('Loss/train_smooth_cmd', train_losses['smooth_cmd'], epoch)
        
        print(f"Epoch {epoch+1}/{cfg.training.epochs} | Train Loss: {train_losses['total']:.4f} "
              f"(Traj: {train_losses['traj']:.4f}, Cmd: {train_losses['cmd']:.4f}, Smooth: {train_losses['smooth']:.4f}, SmoothCmd: {train_losses['smooth_cmd']:.4f})")

        if val_loader is not None:
            val_losses = validate_one_epoch(
                model, val_loader, criterion_traj, criterion_cmd, device, cfg, mode
            )
            writer.add_scalar('Loss/val_total', val_losses['total'], epoch)
            writer.add_scalar('Loss/val_traj', val_losses['traj'], epoch)
            writer.add_scalar('Loss/val_cmd', val_losses['cmd'], epoch)
            writer.add_scalar('Loss/val_smooth', val_losses['smooth'], epoch)
            writer.add_scalar('Loss/val_smooth_cmd', val_losses['smooth_cmd'], epoch)
            
            print(f"→ Validation Loss: {val_losses['total']:.4f} "
                  f"(Traj: {val_losses['traj']:.4f}, Cmd: {val_losses['cmd']:.4f}, Smooth: {val_losses['smooth']:.4f}, SmoothCmd: {val_losses['smooth_cmd']:.4f})")
            current_metric = val_losses['total']
        else:
            current_metric = train_losses['total']

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