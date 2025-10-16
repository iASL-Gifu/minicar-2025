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
from src.model.trajformer import TrajFormerNano


# --- 学習ループ ---
def train_one_epoch(model, dataloader, criterion, optimizer, device):
    model.train()
    total_loss = 0.0

    for batch in tqdm(dataloader, desc="Training"):
        images = batch['image'].to(device)               # (B, C, H, W)
        past_odoms = batch['past_odoms'].to(device)     # (B, past_len, odom_dim)
        future_path = batch['future_path'].to(device)   # (B, future_len, 3)

        optimizer.zero_grad()
        outputs = model(images, past_odoms)
        loss = criterion(outputs, future_path)
        loss.backward()
        optimizer.step()
        total_loss += loss.item()

    return total_loss / len(dataloader)


# --- 検証ループ ---
def validate_one_epoch(model, dataloader, criterion, device):
    model.eval()
    total_loss = 0.0
    with torch.no_grad():
        for batch in tqdm(dataloader, desc="Validation"):
            images = batch['image'].to(device)
            past_odoms = batch['past_odoms'].to(device)
            future_path = batch['future_path'].to(device)

            outputs = model(images, past_odoms)
            loss = criterion(outputs, future_path)
            total_loss += loss.item()

    return total_loss / len(dataloader)


# --- メイン ---
@hydra.main(config_path="config", config_name="train", version_base="1.2")
def main(cfg: DictConfig) -> None:
    print("--- Configuration ---")
    print(OmegaConf.to_yaml(cfg))
    print("---------------------")

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

    # --- Dataset: train ---
    train_dataset = TrajectoryDataset(
        root_dir=Path(train_path),
        transform=TrainTransform(
            height=cfg.dataset.image_height,
            width=cfg.dataset.image_width,
            mode=cfg.dataset.transform_mode
        )
    )
    train_loader = DataLoader(
        train_dataset,
        batch_size=cfg.training.batch_size,
        shuffle=True,
        num_workers=cfg.training.num_workers
    )

    # --- Dataset: test ---
    val_loader = None
    if os.path.exists(test_path):
        val_dataset = TrajectoryDataset(
            root_dir=Path(test_path),
            transform=TestTransform(
                height=cfg.dataset.image_height,
                width=cfg.dataset.image_width,
                mode=cfg.dataset.transform_mode
            )
        )
        val_loader = DataLoader(
            val_dataset,
            batch_size=cfg.training.batch_size,
            shuffle=False,
            num_workers=cfg.training.num_workers
        )
        print(f"✅ Validation dataset found: {test_path}")
    else:
        print(f"⚠️ Validation dataset not found: {test_path}")
        print("→ Skipping validation phase (train loss will be used).")

    # --- モデル定義 ---
    model = TrajFormerNano(
        history_len=cfg.dataset.past_len,
        odom_features=cfg.model.odom_dim,
        future_len=cfg.dataset.future_len,
        image_embedding_dim=cfg.model.image_embedding_dim,
        motion_embedding_dim=cfg.model.motion_embedding_dim,
        transformer_d_model=cfg.model.d_model,
        transformer_nhead=cfg.model.transformer_nhead,
        transformer_num_layers=cfg.model.transformer_num_layers
    ).to(device)


    criterion = nn.SmoothL1Loss()
    optimizer = torch.optim.Adam(model.parameters(), lr=cfg.training.learning_rate)

    best_metric = float('inf')

    # --- 学習ループ ---
    for epoch in range(cfg.training.epochs):
        train_loss = train_one_epoch(model, train_loader, criterion, optimizer, device)
        writer.add_scalar('Loss/train', train_loss, epoch)
        print(f"Epoch {epoch+1}/{cfg.training.epochs} | Train Loss: {train_loss:.4f}")

        if val_loader is not None:
            val_loss = validate_one_epoch(model, val_loader, criterion, device)
            writer.add_scalar('Loss/validation', val_loss, epoch)
            print(f"→ Validation Loss: {val_loss:.4f}")
            current_metric = val_loss
        else:
            current_metric = train_loss

        # --- Save best ---
        if current_metric < best_metric:
            best_metric = current_metric
            torch.save(model.state_dict(), os.path.join(ckpt_dir, 'best_model.pth'))
            print(f"✨ Improved best model (metric={best_metric:.4f}) saved!")

        torch.save(model.state_dict(), os.path.join(ckpt_dir, 'last_model.pth'))

    print("✅ Finished training.")
    writer.close()


if __name__ == '__main__':
    main()
