import torch
import torch.nn as nn

from .trajformer import TrajFormer
from .control_net import ControlFormer

class TrajControlFormer(nn.Module):
    """
    画像と過去オドメトリから未来軌跡を予測し、
    そこからステア角と加速度を出力する統合モデル。
    """

    def __init__(
        self,
        # --- TrajFormer のパラメータ ---
        history_len: int = 10,
        odom_features: int = 8,
        future_len: int = 30,
        image_embedding_dim: int = 128,
        motion_embedding_dim: int = 64,
        transformer_d_model: int = 192,
        transformer_nhead: int = 6,
        transformer_num_layers: int = 2,
        control_d_model: int = 64,
        control_nhead: int = 4,
        control_num_layers: int = 2
    ):
        super().__init__()

        # --- TrajFormer ---
        self.trajformer = TrajFormer(
            history_len=history_len,
            odom_features=odom_features,
            future_len=future_len,
            image_embedding_dim=image_embedding_dim,
            motion_embedding_dim=motion_embedding_dim,
            transformer_d_model=transformer_d_model,
            transformer_nhead=transformer_nhead,
            transformer_num_layers=transformer_num_layers
        )

        # --- ControlFormer (受け取ったパラメータで初期化) ---
        self.control_net = ControlFormer(
            traj_dim=3,  
            d_model=control_d_model,
            nhead=control_nhead,
            num_layers=control_num_layers,
            future_len=future_len
        )

    def forward(self, image: torch.Tensor, past_odoms: torch.Tensor):
        """
        Args:
            image: (B, 3, H, W)
            past_odoms: (B, history_len, odom_features)
        Returns:
            predicted_traj: (B, future_len, 3)
            control_cmd: (B, future_len, 2) [steer, accel]
        """
        predicted_traj = self.trajformer(image, past_odoms)
        control_cmd = self.control_net(predicted_traj)
        return predicted_traj, control_cmd