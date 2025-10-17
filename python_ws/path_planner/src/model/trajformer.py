import torch
import torch.nn as nn
from torchvision.models import mobilenet_v3_small

class TrajFormer(nn.Module):
    """
    軽量Trajectory予測モデル:
    - 画像: MobileNetV3-Small で特徴抽出
    - 過去オドメトリ: GRU で埋め込み
    - 未来軌跡予測: 軽量TransformerDecoder + 線形層
    """

    def __init__(
        self,
        history_len: int = 10,
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

        # --- 1. Image Encoder ---
        self.image_encoder = mobilenet_v3_small(weights='MobileNet_V3_Small_Weights.DEFAULT')
        # stride調整で低解像度入力対応
        self.image_encoder.features[0][0].stride = (1, 1)
        # classifierを特徴量抽出用に置き換え
        num_features = self.image_encoder.classifier[0].in_features
        self.image_encoder.classifier = nn.Sequential(
            nn.Linear(num_features, image_embedding_dim),
            nn.ReLU()
        )

        # --- 2. Motion Encoder (過去オドメトリ) ---
        self.motion_encoder = nn.GRU(
            input_size=odom_features,
            hidden_size=motion_embedding_dim,
            num_layers=2,
            batch_first=True
        )

        # --- 3. Transformer Decoder ---
        decoder_layer = nn.TransformerDecoderLayer(
            d_model=transformer_d_model,
            nhead=transformer_nhead,
            dim_feedforward=transformer_d_model * 2,
            batch_first=True
        )
        self.transformer_decoder = nn.TransformerDecoder(decoder_layer, num_layers=transformer_num_layers)

        # --- 出力層 ---
        self.output_layer = nn.Linear(transformer_d_model, 3)  # (x, y, vx)
        self.query_embed = nn.Parameter(torch.randn(1, future_len, transformer_d_model))

        # --- 画像特徴量とモーション特徴量の結合後を d_model に変換 ---
        self.fusion_layer = nn.Linear(image_embedding_dim + motion_embedding_dim, transformer_d_model)

    def forward(self, image: torch.Tensor, past_odoms: torch.Tensor) -> torch.Tensor:
        """
        Args:
            image: (B, 3, H, W)
            past_odoms: (B, history_len, odom_features)
        Returns:
            predicted_trajectory: (B, future_len, 3)
        """
        # --- 1. 画像特徴 ---
        image_features = self.image_encoder(image)  # (B, image_embedding_dim)

        # --- 2. 過去オドメトリ特徴 ---
        _, motion_features = self.motion_encoder(past_odoms)  # motion_features: (num_layers, B, motion_embedding_dim)
        motion_features = motion_features[-1]  # 最終層の隠れ状態 (B, motion_embedding_dim)

        # --- 3. Fusion ---
        context = torch.cat([image_features, motion_features], dim=1)  # (B, image+motion)
        memory = self.fusion_layer(context).unsqueeze(1)  # (B, 1, transformer_d_model)

        # --- 4. Transformer Decoder ---
        tgt = self.query_embed.repeat(image.size(0), 1, 1)  # (B, future_len, transformer_d_model)
        traj_features = self.transformer_decoder(tgt=tgt, memory=memory)  # (B, future_len, transformer_d_model)

        # --- 5. 出力 ---
        predicted_trajectory = self.output_layer(traj_features)  # (B, future_len, 3)
        return predicted_trajectory

# --- 実行スクリプト ---
if __name__ == "__main__":
    # モデルのインスタンス化 (デフォルト設定)
    model = TrajFormer()
    
    # 学習可能なパラメータ数を計算
    total_params = sum(p.numel() for p in model.parameters() if p.requires_grad)
    
    print(f"モデル: TrajFormer")
    print(f"総パラメータ数: {total_params:,}")
    print(f"総パラメータ数 (M): {total_params / 1_000_000:.2f} M")

    # (オプション) ダミーデータでフォワードパスのテスト
    try:
        # --- ダミーデータの準備 ---
        B = 4  # バッチサイズ
        H, W = 128, 128 # 画像サイズ
        
        # モデルのinit設定値
        history_len = 10
        odom_features = 8
        future_len = 30

        # ダミーテンソル
        dummy_image = torch.randn(B, 3, H, W)
        dummy_past_odoms = torch.randn(B, history_len, odom_features)
        
        # --- フォワードパス実行 ---
        output = model(dummy_image, dummy_past_odoms)
        
        print("\n--- フォワードパス テスト ---")
        print(f"入力 (Image): \t\t{list(dummy_image.shape)}")
        print(f"入力 (Past Odoms): \t{list(dummy_past_odoms.shape)}")
        print(f"出力 (Predicted Traj): \t{list(output.shape)}")
        print(f"期待される出力サイズ: \t[{B}, {future_len}, 3]")
        print("フォワードパス 成功 ✨")

    except Exception as e:
        print(f"\nフォワードパス テスト中にエラーが発生しました: {e}")