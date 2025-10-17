import torch
import torch.nn as nn

class ControlNet(nn.Module):
    """
    軌跡予測結果からステア角・加速度を推定する軽量制御ネットワーク
    """
    def __init__(self, future_len=30, traj_dim=3, hidden_dim=128):
        super().__init__()
        self.model = nn.Sequential(
            nn.Flatten(),  # (B, future_len * traj_dim)
            nn.Linear(future_len * traj_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, 2)  # 出力: [steer, accel]
        )

    def forward(self, predicted_traj: torch.Tensor) -> torch.Tensor:
        """
        Args:
            predicted_traj: (B, future_len, 3)
        Returns:
            control_cmd: (B, 2) -> [steer, accel]
        """
        return self.model(predicted_traj)
