import torch
import torch.nn as nn
import math

class PositionalEncoding(nn.Module):
    """ (B, T, D) バッチファースト対応 Positional Encoding """
    def __init__(self, d_model: int, dropout: float = 0.1, max_len: int = 50):
        super().__init__()
        self.dropout = nn.Dropout(p=dropout)

        position = torch.arange(max_len).unsqueeze(1)
        div_term = torch.exp(torch.arange(0, d_model, 2) * (-math.log(10000.0) / d_model))
        
        pe = torch.zeros(max_len, d_model)
        pe.requires_grad = False
        pe[:, 0::2] = torch.sin(position * div_term)
        pe[:, 1::2] = torch.cos(position * div_term)
        
        self.register_buffer('pe', pe.unsqueeze(0))

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = x + self.pe[:, :x.size(1), :]
        return self.dropout(x)

class ControlFormer(nn.Module):
    """
    Transformerベース制御ネット (独立GRU + Causal Mask)
    入力: (B, T, 3) 予測軌跡 + (B, history_len, odom_features)
    出力: (B, T, 2) 制御シーケンス
    """
    def __init__(self, 
                 traj_dim: int = 3, 
                 odom_features: int = 8,
                 motion_embedding_dim: int = 64,
                 d_model: int = 64, 
                 nhead: int = 4, 
                 num_layers: int = 2, 
                 future_len: int = 30
                ):
        super().__init__()
        
        # このControlNet専用のMotion Encoder (GRU)
        self.motion_encoder = nn.GRU(
            input_size=odom_features,
            hidden_size=motion_embedding_dim,
            num_layers=1,
            batch_first=True
        )
        
        self.input_proj = nn.Linear(traj_dim, d_model)
        
        # Odom特徴量 (このNetのGRU出力) をd_modelに射影
        self.odom_proj = nn.Linear(motion_embedding_dim, d_model)
        
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

    def forward(self, predicted_traj: torch.Tensor, past_odoms: torch.Tensor):
        """
        Args:
            predicted_traj (torch.Tensor): (B, T, 3)
            past_odoms (torch.Tensor): (B, history_len, odom_features)
        Returns:
            torch.Tensor: (B, T, 2)
        """
        
        # 1. Causal Mask (先読み防止マスク) の生成
        T = predicted_traj.size(1) 
        causal_mask = nn.Transformer.generate_square_subsequent_mask(T).to(predicted_traj.device)
        
        # 2. 軌跡特徴量
        x = self.input_proj(predicted_traj) 
        x = self.pos_encoder(x)
        
        # 3. Odom特徴量 (このNetのGRUで独自に計算)
        _, motion_features = self.motion_encoder(past_odoms)
        motion_features = motion_features[-1] # (B, motion_embedding_dim)
        
        odom_embed = self.odom_proj(motion_features).unsqueeze(1) # (B, 1, d_model)
        
        # 全ての時刻(T)に現在状態(Odom)の特徴量を足し込む
        x = x + odom_embed 
        
        # 4. Transformer Encoder (Causal Mask適用)
        x = self.encoder(x, mask=causal_mask)
        
        return self.fc(x)