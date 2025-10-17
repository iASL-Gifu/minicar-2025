import torch
import torch.nn as nn

class ControlFormer(nn.Module):
    """
    Transformerベース制御ネット
    """
    def __init__(self, traj_dim=3, d_model=64, nhead=4, num_layers=2):
        super().__init__()
        self.input_proj = nn.Linear(traj_dim, d_model)
        encoder_layer = nn.TransformerEncoderLayer(
            d_model=d_model,
            nhead=nhead,
            dim_feedforward=d_model * 2,
            batch_first=True
        )
        self.encoder = nn.TransformerEncoder(encoder_layer, num_layers=num_layers)

        self.pool = nn.AdaptiveAvgPool1d(1)  # 時系列方向を統合
        self.fc = nn.Sequential(
            nn.Linear(d_model, d_model),
            nn.ReLU(),
            nn.Linear(d_model, 2)  # [steer, accel]
        )

    def forward(self, predicted_traj):
        x = self.input_proj(predicted_traj)
        x = self.encoder(x)  # (B, T, d_model)
        return self.fc(x)
