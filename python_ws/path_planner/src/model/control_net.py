import torch
import torch.nn as nn

class ControlFormer(nn.Module):
    """
    Transformerベース制御ネット
    入力: (B, T, 3) の予測軌跡
    出力: (B, T, 2) の制御シーケンス
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

        self.fc = nn.Sequential(
            nn.Linear(d_model, d_model),
            nn.ReLU(),
            nn.Linear(d_model, 2)  # [steer, accel]
        )

    def forward(self, predicted_traj):
        """
        Args:
            predicted_traj (torch.Tensor): (B, T, 3)
        Returns:
            torch.Tensor: (B, T, 2)
        """
        x = self.input_proj(predicted_traj) # (B, T, d_model)
        x = self.encoder(x)                 # (B, T, d_model)
        
        return self.fc(x)                   # (B, T, 2)