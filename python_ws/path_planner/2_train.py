import torch
import torch.nn as nn
from torch.utils.data import DataLoader
from tqdm import tqdm
import hydra
from omegaconf import DictConfig, OmegaConf
from pathlib import Path
import os
from torch.utils.tensorboard import SummaryWriter

from src.data.dataset import TrajectoryDataset   
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


# --- 学習ループ (mode引数を追加) ---
def train_one_epoch(model, dataloader, criterion_traj, criterion_cmd, optimizer, device, cfg, mode):
    model.train()
    total_loss, total_traj_loss, total_cmd_loss, total_smooth_loss = 0.0, 0.0, 0.0, 0.0

    # modeに応じてモデルの特定の部分を train() に設定
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
        
        # --- modeに応じてフォワードパスと損失計算 ---
        if mode == 'all':
            predicted_traj, predicted_cmd = model(images, past_odoms)
            loss_traj = criterion_traj(predicted_traj, future_path)
            loss_cmd = criterion_cmd(predicted_cmd, future_cmd)
            loss_smooth = get_kinematic_loss(predicted_traj)
            
            loss = (cfg.training.loss_weights.traj * loss_traj) + \
                   (cfg.training.loss_weights.cmd * loss_cmd) + \
                   (cfg.training.loss_weights.smooth * loss_smooth)
        
        elif mode == 'path_generation':
            predicted_traj = model.trajformer(images, past_odoms) # TrajFormerのみ実行
            loss_traj = criterion_traj(predicted_traj, future_path)
            loss_smooth = get_kinematic_loss(predicted_traj)
            loss_cmd = torch.tensor(0.0, device=device) # ログ用
            
            loss = (cfg.training.loss_weights.traj * loss_traj) + \
                   (cfg.training.loss_weights.smooth * loss_smooth)
        
        elif mode == 'path_follow':
            # ★重要: 入力は予測軌道ではなく、正解軌道
            predicted_cmd = model.control_net(future_path) 
            loss_cmd = criterion_cmd(predicted_cmd, future_cmd)
            loss_traj = torch.tensor(0.0, device=device) # ログ用
            loss_smooth = torch.tensor(0.0, device=device) # ログ用
            
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


# --- 検証ループ (mode引数を追加) ---
def validate_one_epoch(model, dataloader, criterion_traj, criterion_cmd, device, cfg, mode):
    model.eval() # eval() はモデル全体でOK
    total_loss, total_traj_loss, total_cmd_loss, total_smooth_loss = 0.0, 0.0, 0.0, 0.0
    
    with torch.no_grad():
        for batch in tqdm(dataloader, desc=f"Validation ({mode})"):
            images = batch['image'].to(device)
            past_odoms = batch['past_odoms'].to(device)
            future_path = batch['future_path'].to(device)
            future_cmd = batch['future_cmd'].to(device)

            # --- modeに応じてフォワードパスと損失計算 ---
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


# --- メイン ---
@hydra.main(config_path="config", config_name="train", version_base="1.2")
def main(cfg: DictConfig) -> None:
    print("--- Configuration ---")
    print(OmegaConf.to_yaml(cfg))
    print("---------------------")

    # --- 【新規】モードの取得と検証 ---
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
    train_dataset = TrajectoryDataset(
        root_dir=Path(train_path),
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
        val_dataset = TrajectoryDataset(
            root_dir=Path(test_path),
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
    # TrajControlFormerは常に全体をロードする
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

    criterion_traj = nn.SmoothL1Loss()
    criterion_cmd = nn.SmoothL1Loss()
    
    # --- modeに応じてオプティマイザの対象パラメータ ---
    if mode == 'all':
        parameters = model.parameters()
        print("Optimizing: ALL parameters.")
    elif mode == 'path_generation':
        parameters = model.trajformer.parameters()
        # 念のため、もう一方をフリーズ
        for param in model.control_net.parameters():
            param.requires_grad = False
        print("Optimizing: ONLY TrajFormer parameters.")
    elif mode == 'path_follow':
        parameters = model.control_net.parameters()
        # 念のため、もう一方をフリーズ
        for param in model.trajformer.parameters():
            param.requires_grad = False
        print("Optimizing: ONLY ControlFormer parameters.")
        
    optimizer = torch.optim.Adam(parameters, lr=cfg.training.learning_rate)

    best_metric = float('inf')

    # --- 学習ループ ---
    for epoch in range(cfg.training.epochs):
        
        # modeを渡す
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
            # modeを渡す
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

        # last_modelも同様に
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