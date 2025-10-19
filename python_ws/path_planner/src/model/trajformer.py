import torch
import torch.nn as nn
from torchvision.models import shufflenet_v2_x0_5, ShuffleNet_V2_X0_5_Weights

class TrajFormer(nn.Module):
    """
    軽量Trajectory予測モデル:
    - 画像: ShuffleNetV2 x0.5
    - 過去オドメトリ: GRU
    - 未来軌跡予測: TransformerDecoder
    """

    def __init__(
        self,
        history_len: int = 10,
        odom_features: int = 8,
        future_len: int = 30,
        image_embedding_dim: int = 128,
        motion_embedding_dim: int = 64,
        transformer_d_model: int = 192,
        transformer_nhead: int = 6,
        transformer_num_layers: int = 2
    ):
        super().__init__()
        self.future_len = future_len

        # 1. Image Encoder (ShuffleNetV2 x0.5)
        self.image_encoder = shufflenet_v2_x0_5(weights=ShuffleNet_V2_X0_5_Weights.DEFAULT)
        self.image_encoder.conv1[0].stride = (1, 1) 
        
        num_features = self.image_encoder.fc.in_features
        self.image_encoder.fc = nn.Sequential(
            nn.Linear(num_features, image_embedding_dim),
            nn.ReLU()
        )

        # 2. Motion Encoder (GRU)
        self.motion_encoder = nn.GRU(
            input_size=odom_features,
            hidden_size=motion_embedding_dim,
            num_layers=1,
            batch_first=True
        )

        # 3. Transformer Decoder
        decoder_layer = nn.TransformerDecoderLayer(
            d_model=transformer_d_model,
            nhead=transformer_nhead,
            dim_feedforward=transformer_d_model * 2,
            batch_first=True
        )
        self.transformer_decoder = nn.TransformerDecoder(decoder_layer, num_layers=transformer_num_layers)

        # 4. Output & Query
        self.output_layer = nn.Linear(transformer_d_model, 3)  # (x, y, vx)
        self.query_embed = nn.Parameter(torch.randn(1, future_len, transformer_d_model))

        # 5. Fusion Layer
        self.fusion_layer = nn.Linear(image_embedding_dim + motion_embedding_dim, transformer_d_model)

    def forward(self, image: torch.Tensor, past_odoms: torch.Tensor) -> torch.Tensor:
        """
        Args:
            image: (B, 3, H, W)
            past_odoms: (B, history_len, odom_features)
        Returns:
            predicted_trajectory: (B, future_len, 3)
        """
        # 1. 画像特徴
        image_features = self.image_encoder(image)

        # 2. 過去オドメトリ特徴
        _, motion_features = self.motion_encoder(past_odoms)
        motion_features = motion_features[-1]  # 最終層の隠れ状態

        # 3. Fusion
        context = torch.cat([image_features, motion_features], dim=1)
        memory = self.fusion_layer(context).unsqueeze(1)

        # 4. Transformer Decoder
        tgt = self.query_embed.repeat(image.size(0), 1, 1)
        traj_features = self.transformer_decoder(tgt=tgt, memory=memory)

        # 5. 出力
        predicted_trajectory = self.output_layer(traj_features)
        
        return predicted_trajectory