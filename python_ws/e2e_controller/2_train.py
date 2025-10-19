import torch
import torch.nn as nn
from torch.utils.data import DataLoader
from tqdm import tqdm
import hydra
from omegaconf import DictConfig, OmegaConf
import os
from torch.utils.tensorboard import SummaryWriter

from src.data.dataset import MultiSequenceDataset  
from src.data.transform import TrainTransform, TestTransform
from src.model.pilotnet import PilotNet


# =========================================================
# 学習1エポック 
# =========================================================
def train_one_epoch(model, dataloader, criterion, optimizer, device):
    model.train()
    total_loss = 0.0
    for batch in tqdm(dataloader, desc="Training"):
        images = batch['image'].to(device)
        steers = batch['steer']
        speeds = batch['speed']

        b, s, c, h, w = images.shape
        inputs = images.view(b * s, c, h, w)
        labels = torch.stack([
            steers.view(b * s),
            speeds.view(b * s)
        ], dim=-1).to(device)

        optimizer.zero_grad()
        outputs = model(inputs)
        loss = criterion(outputs, labels)
        loss.backward()
        optimizer.step()
        total_loss += loss.item()

    return total_loss / len(dataloader)


# =========================================================
# 検証1エポック
# =========================================================
def validate_one_epoch(model, dataloader, criterion, device):
    model.eval()
    total_loss = 0.0
    with torch.no_grad():
        for batch in tqdm(dataloader, desc="Validation"):
            images = batch['image'].to(device)
            steers = batch['steer']
            speeds = batch['speed']

            b, s, c, h, w = images.shape
            inputs = images.view(b * s, c, h, w)
            labels = torch.stack([
                steers.view(b * s),
                speeds.view(b * s)
            ], dim=-1).to(device)

            outputs = model(inputs)
            loss = criterion(outputs, labels)
            total_loss += loss.item()

    return total_loss / len(dataloader)


# =========================================================
# メイン
# =========================================================
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

    # =====================================================
    # Dataset: train
    # =====================================================
    train_dataset = MultiSequenceDataset(  
        base_dir=train_path,
        seq_len=cfg.dataset.sequence_length,
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

    # =====================================================
    # Dataset: validation
    # =====================================================
    val_loader = None
    if os.path.exists(test_path):
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
            batch_size=cfg.training.batch_size,
            shuffle=False,
            num_workers=cfg.training.num_workers
        )
        print(f"✅ Validation dataset found: {test_path}")
    else:
        print(f"⚠️ Validation dataset not found: {test_path}")
        print("→ Skipping validation phase (train loss will be used).")

    # =====================================================
    # モデル・損失関数・オプティマイザ
    # =====================================================
    model = PilotNet(num_outputs=cfg.model.num_outputs).to(device)
    criterion = nn.SmoothL1Loss()
    optimizer = torch.optim.Adam(model.parameters(), lr=cfg.training.learning_rate)

    best_metric = float('inf')

    resume_ckpt_path = cfg.get('resume_ckpt_path', None)

    if resume_ckpt_path:
        resume_ckpt_path_abs = hydra.utils.to_absolute_path(resume_ckpt_path)
        if os.path.exists(resume_ckpt_path_abs):
            print(f"🔄 Loading weights from: {resume_ckpt_path_abs}")
            try:
                # 重み（state_dict）を直接読み込む
                weights = torch.load(resume_ckpt_path_abs, map_location=device)
                
                if isinstance(weights, dict) and 'model_state_dict' in weights:

                    model.load_state_dict(weights['model_state_dict'])
                    print("→ Loaded 'model_state_dict' from checkpoint dictionary.")
                elif isinstance(weights, dict):
                     # state_dict が直接保存されている場合
                    model.load_state_dict(weights)
                    print("→ Loaded weights (state_dict) directly.")
                else:
                    print(f"⚠️ Checkpoint format not recognized (Type: {type(weights)}). Starting from scratch.")

            except Exception as e:
                print(f"⚠️ Failed to load weights: {e}")
                print("→ Starting training from scratch.")
        else:
            print(f"⚠️ Checkpoint path specified but not found: {resume_ckpt_path_abs}")
            print("→ Starting training from scratch.")
    else:
        print("🚀 Starting training from scratch.")



    # =====================================================
    # 学習ループ
    # =====================================================
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

        # === Best model 保存 ===
        if current_metric < best_metric:
            best_metric = current_metric
            
            torch.save(model.state_dict(), os.path.join(ckpt_dir, 'best_model.pth'))
            print(f"✨ Improved best model (metric={best_metric:.4f}) saved!")

        torch.save(model.state_dict(), os.path.join(ckpt_dir, 'last_model.pth'))

    print("✅ Finished training.")
    writer.close()


if __name__ == '__main__':
    main()