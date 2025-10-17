import torch.nn as nn
from trajformer import TrajFormerNano
from control_net import ControlNet

class TrajFormerWithControl(nn.Module):
    """
    画像・過去オドメトリから未来軌跡を予測し、
    そこからステア角と加速度を出力する統合モデル。
    """
    def __init__(self, **trajformer_kwargs):
        super().__init__()
        self.trajformer = TrajFormerNano(**trajformer_kwargs)
        self.control_net = ControlNet(
            future_len=trajformer_kwargs.get("future_len", 30)
        )

    def forward(self, image, past_odoms):
        predicted_traj = self.trajformer(image, past_odoms)  # (B, future_len, 3)
        control_cmd = self.control_net(predicted_traj)        # (B, 2)
        return predicted_traj, control_cmd
