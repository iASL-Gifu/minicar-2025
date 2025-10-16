import torch
import torch.nn as nn
from torchvision.models import mobilenet_v3_small

class TrajFormerNano(nn.Module):
    def __init__(
        self,
        # --- (引数は前回と同様) ---
        history_len: int = 5,
        odom_features: int = 8,
        future_len: int = 10,
        image_embedding_dim: int = 128,
        motion_embedding_dim: int = 64,
        transformer_d_model: int = 192,
        transformer_nhead: int = 6, 
        transformer_num_layers: int = 2
    ):
        super().__init__()
        self.future_len = future_len

        # 1. Image Encoder: MobileNetV3-Small
        self.image_encoder = mobilenet_v3_small(weights='MobileNet_V3_Small_Weights.DEFAULT')

        # --- 最適化ポイント ---
        # 低解像度入力のため、最初の層のstrideを2から1に変更し、空間情報の損失を抑える
        # オリジナル: self.image_encoder.features[0][0] = Conv2d(3, 16, kernel_size=(3, 3), stride=(2, 2), padding=(1, 1), bias=False)
        self.image_encoder.features[0][0].stride = (1, 1)
        
        # 最後の分類層を特徴量抽出用に変更
        num_features = self.image_encoder.classifier[0].in_features
        self.image_encoder.classifier = nn.Sequential(
            nn.Linear(num_features, image_embedding_dim),
            nn.ReLU()
        )

        # 2. Motion Encoder: GRU 
        self.motion_encoder = nn.GRU(
            input_size=odom_features,
            hidden_size=motion_embedding_dim,
            num_layers=2,
            batch_first=True
        )

        # 3. Decoder: 軽量Transformer
        decoder_layer = nn.TransformerDecoderLayer(
            d_model=transformer_d_model,
            nhead=transformer_nhead, 
            dim_feedforward=transformer_d_model * 2,
            batch_first=True
        )
        self.transformer_decoder = nn.TransformerDecoder(decoder_layer, num_layers=transformer_num_layers)
        
        self.output_layer = nn.Linear(transformer_d_model, 3) # 出力は (x, y, vx) の3次元
        self.query_embed = nn.Parameter(torch.randn(1, future_len, transformer_d_model))

    def forward(self, image: torch.Tensor, past_odoms: torch.Tensor) -> torch.Tensor:
        """
        Args:
            image (torch.Tensor): (B, 3, 120, 160) の画像バッチ
            past_odoms (torch.Tensor): (B, history_len, odom_features) の過去オドメトリバッチ
        """
        # (B, 3, 120, 160) -> (B, image_embedding_dim)
        image_features = self.image_encoder(image)
        
        # (B, history_len, odom_features) -> (B, motion_embedding_dim)
        _, motion_features = self.motion_encoder(past_odoms)
        motion_features = motion_features[-1]

        # Fusion
        context = torch.cat([image_features, motion_features], dim=1)
        
        # Transformer Decoder
        memory = context.unsqueeze(1)
        tgt = self.query_embed.repeat(image.size(0), 1, 1)
        trajectory_features = self.transformer_decoder(tgt=tgt, memory=memory)

        predicted_trajectory = self.output_layer(trajectory_features)

        return predicted_trajectory