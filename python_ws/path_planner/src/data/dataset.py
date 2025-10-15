from typing import Callable
import torch
from torch.utils.data import Dataset
from torchvision import transforms
import cv2
from pathlib import Path
import numpy as np

class TrajectoryDataset(Dataset):
    """
    複数の記録ディレクトリから、画像、過去のオドメトリ、未来の軌跡を読み込むための
    PyTorch Datasetクラス。
    """
    def __init__(self, root_dir: Path, transform: Callable | None = None):
        """
        Args:
            root_dir (Path): 複数のデータセットディレクトリが格納されているルートディレクトリ。
                             (例: ./my_dataset/)
            transform (Callable, optional): 画像に適用する前処理。デフォルトはNone。
        """
        super().__init__()
        
        self.root_dir = root_dir
        self.transform = transform
        
        self.recordings = []
        self.sample_map = []

        # ルートディレクトリ内の各記録ディレクトリを探索
        recording_dirs = sorted([p for p in self.root_dir.iterdir() if p.is_dir()])
        print(f"[INFO] Found {len(recording_dirs)} potential recording directories in {self.root_dir}")

        for recording_idx, recording_path in enumerate(recording_dirs):
            image_dir = recording_path / "images"
            future_paths_file = recording_path / "paths.npy"
            past_odoms_file = recording_path / "past_odoms.npy"

            # 必要なファイルが揃っているかチェック
            if not all([image_dir.is_dir(), future_paths_file.exists(), past_odoms_file.exists()]):
                print(f"[WARN] Skipping {recording_path.name}: required file is missing.")
                continue

            image_files = sorted(list(image_dir.glob("*.png")))
            future_paths = np.load(future_paths_file)
            past_odoms = np.load(past_odoms_file)
            
            # 各データのサンプル数が一致しているかチェック
            num_samples = len(image_files)
            if not (num_samples == len(future_paths) == len(past_odoms)):
                print(f"[WARN] Skipping {recording_path.name}: data length mismatch.")
                continue

            # 有効な記録データをリストに格納
            self.recordings.append({
                'image_files': image_files,
                'future_paths': future_paths,
                'past_odoms': past_odoms,
            })
            
            # グローバルインデックスと、(記録ディレクトリ, ローカルインデックス) を対応付けるマップを作成
            for local_idx in range(num_samples):
                self.sample_map.append((recording_idx, local_idx))
                
            print(f"[INFO] Loaded {recording_path.name} with {num_samples} samples.")

    def __len__(self) -> int:
        """データセットの総サンプル数を返す。"""
        return len(self.sample_map)

    def __getitem__(self, idx: int) -> dict[str, torch.Tensor]:
        """
        指定されたグローバルインデックスのデータを1サンプル取得する。

        Args:
            idx (int): 取得するデータのグローバルインデックス。

        Returns:
            dict[str, torch.Tensor]: 各データを含む辞書。
        """
        # グローバルインデックスから、どの記録の何番目のデータかを解決
        recording_idx, local_idx = self.sample_map[idx]
        
        # 対応する記録データを取得
        recording_data = self.recordings[recording_idx]
        
        # 1. 画像を読み込む
        image_path = recording_data['image_files'][local_idx]
        image = cv2.imread(str(image_path))
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        # 2. 画像にTransformを適用
        if self.transform:
            image = self.transform(image)
        
        # 3. 対応する過去と未来のデータを取得
        past_odom = recording_data['past_odoms'][local_idx]
        future_path = recording_data['future_paths'][local_idx]
        
        # 4. NumPy配列をPyTorchテンソルに変換
        past_odom_tensor = torch.tensor(past_odom, dtype=torch.float32)
        future_path_tensor = torch.tensor(future_path, dtype=torch.float32)
        
        return {
            'image': image,
            'past_odoms': past_odom_tensor,
            'future_path': future_path_tensor
        }