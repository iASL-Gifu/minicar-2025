import torch
import torch.nn as nn

class PilotNet(nn.Module):
    """
    NVIDIAのPilotNetアーキテクチャをPyTorchで実装したモデル。
    入力: (バッチサイズ, 3, 120, 160) の正規化された画像テンソル
    出力: (バッチサイズ, 2) の [操舵角, 速度] テンソル
    """
    def __init__(self, num_outputs=2):
        super().__init__()

        # --- 畳み込み層 (特徴抽出) ---
        self.features = nn.Sequential(
            # 1. Conv層: 3ch -> 24ch, Kernel 5x5, Stride 2
            nn.Conv2d(in_channels=3, out_channels=24, kernel_size=5, stride=2),
            nn.ReLU(),
            # 2. Conv層: 24ch -> 36ch, Kernel 5x5, Stride 2
            nn.Conv2d(in_channels=24, out_channels=36, kernel_size=5, stride=2),
            nn.ReLU(),
            # 3. Conv層: 36ch -> 48ch, Kernel 5x5, Stride 2
            nn.Conv2d(in_channels=36, out_channels=48, kernel_size=5, stride=2),
            nn.ReLU(),
            # 4. Conv層: 48ch -> 64ch, Kernel 3x3, Stride 1
            nn.Conv2d(in_channels=48, out_channels=64, kernel_size=3, stride=1),
            nn.ReLU(),
            # 5. Conv層: 64ch -> 64ch, Kernel 3x3, Stride 1
            nn.Conv2d(in_channels=64, out_channels=64, kernel_size=3, stride=1),
            nn.ReLU()
        )

        # --- 全結合層 (操舵と速度の決定) ---
        self.classifier = nn.Sequential(
            nn.Flatten(),
            # ★ 1. FC層: 入力ユニット数を変更
            # (入力サイズは 64ch * 8H * 13W = 6656 となります)
            nn.Linear(in_features=6656, out_features=1164),
            nn.ReLU(),
            # 2. FC層: 1164 -> 100
            nn.Linear(in_features=1164, out_features=100),
            nn.ReLU(),
            # 3. FC層: 100 -> 50
            nn.Linear(in_features=100, out_features=50),
            nn.ReLU(),
            # 4. FC層: 50 -> 10
            nn.Linear(in_features=50, out_features=10),
            nn.ReLU(),
            # 5. 出力層: 10 -> 2 (steer, speed)
            nn.Linear(in_features=10, out_features=num_outputs)
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        x = self.features(x)
        output = self.classifier(x)

        if output.shape[1] == 2:
            steer = torch.atan(output[:, 0]) * 2
            speed = output[:, 1] 
            output = torch.stack([steer, speed], dim=1)
        else:
            output = torch.atan(output) * 2
            
        return output
    
class TinyPilotNet(nn.Module):
    """
    PilotNetのアーキテクチャの層の数を減らした（浅くした）モデル。
    畳み込み層を5層から3層に、全結合層を5層から3層に削減しています。

    入力: (バッチサイズ, 3, 120, 160) の正規化された画像テンソル
    出力: (バッチサイズ, 2) の [操舵角, 速度] テンソル
    """
    def __init__(self, num_outputs=2):
        super().__init__()

        # --- 畳み込み層 (5層 -> 3層) ---
        # ストライドが1の後半2層を削除
        self.features = nn.Sequential(
            # 1. Conv層: 3ch -> 24ch, Kernel 5x5, Stride 2
            nn.Conv2d(in_channels=3, out_channels=24, kernel_size=5, stride=2),
            nn.ReLU(),
            # 2. Conv層: 24ch -> 36ch, Kernel 5x5, Stride 2
            nn.Conv2d(in_channels=24, out_channels=36, kernel_size=5, stride=2),
            nn.ReLU(),
            # 3. Conv層: 36ch -> 48ch, Kernel 5x5, Stride 2
            nn.Conv2d(in_channels=36, out_channels=48, kernel_size=5, stride=2),
            nn.ReLU(),
        )

        # --- 全結合層 (5層 -> 3層) ---
        # 中間の1164 -> 100 と 50 -> 10 の層を削除
        self.classifier = nn.Sequential(
            nn.Flatten(),
            # ★ 1. FC層: 入力ユニット数を変更
            # (入力サイズは 48ch * 12H * 17W = 9792 となります)
            # (入力 120x160 -> C1(58x78) -> C2(27x37) -> C3(12x17))
            nn.Linear(in_features=9792, out_features=100),
            nn.ReLU(),
            # 2. FC層: 100 -> 50
            nn.Linear(in_features=100, out_features=50),
            nn.ReLU(),
            # 3. 出力層: 50 -> 2 (steer, speed)
            nn.Linear(in_features=50, out_features=num_outputs)
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        順伝播。
        forwardロジックは元のPilotNetと同じです。
        """
        x = self.features(x)
        output = self.classifier(x)

        # 出力が2次元（操舵、速度）の場合の処理
        if output.shape[1] == 2:
            # 操舵角（1列目）にatanを適用
            steer = torch.atan(output[:, 0]) * 2
            # 速度（2列目）はそのまま
            speed = output[:, 1] 
            output = torch.stack([steer, speed], dim=1)
        
        # 出力が1次元（操舵のみなど）の場合の処理
        else:
            output = torch.atan(output) * 2
            
        return output