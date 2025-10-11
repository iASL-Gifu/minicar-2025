import torch
import torch.nn as nn
from torch.utils.data import DataLoader, random_split, Subset
from tqdm import tqdm
import hydra
from omegaconf import DictConfig, OmegaConf
import os
from torch.utils.tensorboard import SummaryWriter

from src.data.dataset import RecordingSequenceDataset
from src.data.transform import TrainTransform, TestTransform
from src.model.pilotnet import PilotNet

# --- 学習・検証ループ関数 ---
def train_one_epoch(model, dataloader, criterion, optimizer, device):
    model.train()
    total_loss = 0.0
    for inputs, labels in tqdm(dataloader, desc="Training"):
        inputs, labels = inputs.to(device), labels.to(device)
        optimizer.zero_grad()
        outputs = model(inputs)
        loss = criterion(outputs, labels)
        loss.backward()
        optimizer.step()
        total_loss += loss.item()
    return total_loss / len(dataloader)

def validate_one_epoch(model, dataloader, criterion, device):
    model.eval()
    total_loss = 0.0
    with torch.no_grad():
        for inputs, labels in tqdm(dataloader, desc="Validation"):
            inputs, labels = inputs.to(device), labels.to(device)
            outputs = model(inputs)
            loss = criterion(outputs, labels)
            total_loss += loss.item()
    return total_loss / len(dataloader)

# --- メイン実行関数 ---
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

    data_path = hydra.utils.to_absolute_path(cfg.data_path)
    full_dataset = RecordingSequenceDataset(root_dir=data_path, sequence_length=cfg.dataset.sequence_length)
    
    train_size = int(cfg.dataset.train_val_split_ratio * len(full_dataset))
    val_size = len(full_dataset) - train_size
    train_indices, val_indices = random_split(range(len(full_dataset)), [train_size, val_size])
    
    train_dataset = Subset(full_dataset, train_indices)
    train_dataset.dataset.transform = TrainTransform(height=cfg.dataset.image_height, width=cfg.dataset.image_width)
    val_dataset = Subset(full_dataset, val_indices)
    val_dataset.dataset.transform = TestTransform(height=cfg.dataset.image_height, width=cfg.dataset.image_width)

    train_loader = DataLoader(train_dataset, batch_size=cfg.training.batch_size, shuffle=True, num_workers=cfg.training.num_workers)
    val_loader = DataLoader(val_dataset, batch_size=cfg.training.batch_size, shuffle=False, num_workers=cfg.training.num_workers)
    
    model = PilotNet(num_outputs=cfg.model.num_outputs).to(device)
    criterion = nn.SmoothL1Loss()
    optimizer = torch.optim.Adam(model.parameters(), lr=cfg.training.learning_rate)
    
    best_val_loss = float('inf')
    
    for epoch in range(cfg.training.epochs):
        train_loss = train_one_epoch(model, train_loader, criterion, optimizer, device)
        val_loss = validate_one_epoch(model, val_loader, criterion, device)
        
        print(f"Epoch {epoch+1}/{cfg.training.epochs} | Train Loss: {train_loss:.4f} | Val Loss: {val_loss:.4f}")
        
        writer.add_scalar('Loss/train', train_loss, epoch)
        writer.add_scalar('Loss/validation', val_loss, epoch)

        if val_loss < best_val_loss:
            best_val_loss = val_loss
            torch.save(model.state_dict(), os.path.join(ckpt_dir, 'best_model.pth'))
            print(f"✨ Validation loss improved to {val_loss:.4f}. Saving best model to {ckpt_dir}")

    torch.save(model.state_dict(), os.path.join(ckpt_dir, 'last_model.pth'))
    print(f"Finished training. Saving last model to {ckpt_dir}")
    
    writer.close()

if __name__ == '__main__':
    main()