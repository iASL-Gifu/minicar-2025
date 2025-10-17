from typing import Callable, Dict
import torch
from torch.utils.data import Dataset
import cv2
from pathlib import Path
import numpy as np

class TrajectoryDataset(Dataset):
    """
    複数の記録ディレクトリから、サンプルごとの
    (画像, 過去オドメトリ, 未来軌跡, 未来コマンド) を読み込むDatasetクラス。
    """
    def __init__(self, root_dir: Path, transform: Callable | None = None):
        """
        Args:
            root_dir (Path): 複数のデータセットディレクトリが格納されているルートディレクトリ。
            transform (Callable, optional): 【画像にのみ】適用する前処理。
        """
        super().__init__()
        
        self.root_dir = root_dir
        self.transform = transform # 画像にのみ適用される
        
        self.sample_paths = [] 

        recording_dirs = sorted([p for p in self.root_dir.iterdir() if p.is_dir()])
        print(f"[INFO] Found {len(recording_dirs)} potential recording directories in {self.root_dir}")

        for recording_path in recording_dirs:
            image_dir = recording_path / "images"
            if not image_dir.is_dir():
                print(f"[WARN] Skipping {recording_path.name}: 'images' directory not found.")
                continue

            image_files = sorted(list(image_dir.glob("*.png")))
            num_found = 0
            for img_path in image_files:
                sample_stem = img_path.stem
                
                past_odom_file = recording_path / f"past_odoms_{sample_stem}.npy"
                future_traj_file = recording_path / f"future_trajectory_{sample_stem}.npy"
                future_cmd_file = recording_path / f"future_cmds_{sample_stem}.npy"

                if all([past_odom_file.exists(), future_traj_file.exists(), future_cmd_file.exists()]):
                    self.sample_paths.append({
                        'image': img_path,
                        'past_odoms': past_odom_file,
                        'future_path': future_traj_file,
                        'future_cmd': future_cmd_file
                    })
                    num_found += 1
                
            if num_found > 0:
                print(f"[INFO] Loaded {recording_path.name} with {num_found} valid samples.")
            else:
                print(f"[WARN] Skipping {recording_path.name}: No valid samples found.")

    def __len__(self) -> int:
        return len(self.sample_paths)

    def __getitem__(self, idx: int) -> Dict[str, torch.Tensor]:
        """
        指定されたグローバルインデックスのデータを1サンプル取得する。
        """
        paths = self.sample_paths[idx]
        
        try:
            # 1. 画像を読み込む (RGB)
            image_np = cv2.imread(str(paths['image']))
            if image_np is None:
                raise IOError(f"Failed to read image: {paths['image']}")
            image_np = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)
            
            # 2. 他のNumpyデータを読み込む
            past_odoms_np = np.load(paths['past_odoms'])
            future_path_np = np.load(paths['future_path'])
            future_cmd_np = np.load(paths['future_cmd'])
            
        except Exception as e:
            print(f"[ERROR] Failed to load data for index {idx} (path: {paths['image']}). Error: {e}")
            return self.__getitem__((idx + 1) % len(self))

        # 3. 画像にのみTransformを適用
        if self.transform:
            image_tensor = self.transform(image_np)
        else:
            # Transformがない場合は、手動で (C, H, W) のTensorに変換
            image_tensor = torch.from_numpy(image_np.transpose(2, 0, 1)).float() / 255.0

        # 4. 他のデータをTensorに変換
        past_odoms_tensor = torch.tensor(past_odoms_np, dtype=torch.float32)
        future_path_tensor = torch.tensor(future_path_np, dtype=torch.float32)
        future_cmd_tensor = torch.tensor(future_cmd_np, dtype=torch.float32)

        return {
            'image': image_tensor,
            'past_odoms': past_odoms_tensor,
            'future_path': future_path_tensor,
            'future_cmd': future_cmd_tensor
        }