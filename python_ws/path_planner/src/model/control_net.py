import torch
import torch.nn as nn

import math

# (PositionalEncoding クラスを ControlFormer の前に定義)
class PositionalEncoding(nn.Module):
    """ (B, T, D) のバッチファースト入力に対応した Positional Encoding """
    def __init__(self, d_model: int, dropout: float = 0.1, max_len: int = 50):
        super().__init__()
        self.dropout = nn.Dropout(p=dropout)

        position = torch.arange(max_len).unsqueeze(1) # (max_len, 1)
        div_term = torch.exp(torch.arange(0, d_model, 2) * (-math.log(10000.0) / d_model))
        
        # (max_len, d_model) の PE 行列を作成
        pe = torch.zeros(max_len, d_model)
        pe.requires_grad = False
        pe[:, 0::2] = torch.sin(position * div_term)
        pe[:, 1::2] = torch.cos(position * div_term)
        
        # (max_len, d_model) -> (1, max_len, d_model) に変更して register_buffer
        self.register_buffer('pe', pe.unsqueeze(0))

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        Args:
            x: (B, T, d_model)
        """
        # x のシーケンス長 T に合わせて pe をスライス
        x = x + self.pe[:, :x.size(1), :]
        return self.dropout(x)

class ControlFormer(nn.Module):
    """
    Transformerベース制御ネット
    入力: (B, T, 3) の予測軌跡
    出力: (B, T, 2) の制御シーケンス
    """
    def __init__(self, traj_dim=3, d_model=64, nhead=4, num_layers=2, future_len=30): # future_len を追加
        super().__init__()
        self.input_proj = nn.Linear(traj_dim, d_model)
        
        # --- 改善点: Positional Encoding の追加 ---
        self.pos_encoder = PositionalEncoding(d_model, max_len=future_len)
        
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
        
        # --- 改善点: Positional Encoding の適用 ---
        x = self.pos_encoder(x)
        
        x = self.encoder(x)                 # (B, T, d_model)
        
        return self.fc(x)                   # (B, T, 2)