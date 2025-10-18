from typing import Callable, Dict, List, Optional
import torch
from torch.utils.data import Dataset, ConcatDataset
import cv2
from pathlib import Path
import numpy as np


class SequenceDataset(Dataset):
    """
    単一の記録ディレクトリを扱うDataset。
    """
    def __init__(self, recording_path: Path, transform: Callable | None = None):
        self.recording_path = recording_path
        self.transform = transform
        self.sample_paths = []

        image_dir = recording_path / "images"
        if not image_dir.is_dir():
            raise FileNotFoundError(f"[WARN] {recording_path} に 'images' ディレクトリが存在しません。")

        image_files = sorted(list(image_dir.glob("*.png")))
        num_found = 0
        for img_path in image_files:
            stem = img_path.stem
            past_odom_file = recording_path / f"past_odoms_{stem}.npy"
            future_traj_file = recording_path / f"future_trajectory_{stem}.npy"
            future_cmd_file = recording_path / f"future_cmds_{stem}.npy"

            if all([past_odom_file.exists(), future_traj_file.exists(), future_cmd_file.exists()]):
                self.sample_paths.append({
                    'image': img_path,
                    'past_odoms': past_odom_file,
                    'future_path': future_traj_file,
                    'future_cmd': future_cmd_file
                })
                num_found += 1

        if num_found == 0:
            raise RuntimeError(f"[WARN] {recording_path.name} には有効なサンプルが存在しません。")
        else:
            print(f"[INFO] Loaded {recording_path.name} with {num_found} valid samples.")

    def __len__(self):
        return len(self.sample_paths)

    def __getitem__(self, idx: int) -> Dict[str, torch.Tensor]:
        paths = self.sample_paths[idx]
        try:
            image_np = cv2.imread(str(paths['image']))
            if image_np is None:
                raise IOError(f"Failed to read image: {paths['image']}")
            image_np = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)

            past_odoms_np = np.load(paths['past_odoms'])
            future_path_np = np.load(paths['future_path'])
            future_cmd_np = np.load(paths['future_cmd'])

        except Exception as e:
            print(f"[ERROR] Failed to load data at {paths['image']}. Error: {e}")
            return self.__getitem__((idx + 1) % len(self))

        sample_np = {
            'image': image_np,
            'past_odoms': past_odoms_np,
            'future_path': future_path_np,
            'future_cmd': future_cmd_np
        }

        if self.transform:
            return self.transform(sample_np)
        else:
            return {
                'image': torch.from_numpy(image_np.transpose(2, 0, 1)).float() / 255.0,
                'past_odoms': torch.tensor(past_odoms_np, dtype=torch.float32),
                'future_path': torch.tensor(future_path_np, dtype=torch.float32),
                'future_cmd': torch.tensor(future_cmd_np, dtype=torch.float32)
            }


class MultiSequenceDataset(Dataset):
    """
    base_dir 以下を再帰的に探索し、複数 sequence を結合した Dataset。
    特定のシーケンスのみを選択して利用する機能を追加。
    """
    def __init__(
        self,
        base_dir: Path,
        transform: Optional[Callable] = None,
        sequence_indices: Optional[List[int]] = None  
    ):
        self.base_dir = Path(base_dir)
        self.transform = transform

        # 再帰探索でシーケンスディレクトリを収集
        self.recording_dirs = self._find_recording_dirs(self.base_dir)
        if len(self.recording_dirs) == 0:
            raise RuntimeError(f"[ERROR] No valid recording directories found under {self.base_dir}")

        # シーケンス名一覧
        self.sequence_names = [d.name for d in self.recording_dirs]
        print(f"[INFO] Found {len(self.recording_dirs)} sequence directories under {self.base_dir}")
        print("    Sequences:", ", ".join(self.sequence_names))

        # --- 部分利用モード ---
        if sequence_indices is not None:
            # index指定された部分だけ抽出
            self.recording_dirs = [self.recording_dirs[i] for i in sequence_indices]
            print(f"[INFO] Using subset of sequences: {[self.sequence_names[i] for i in sequence_indices]}")

        # --- 各シーケンスをDataset化 ---
        datasets = []
        for d in self.recording_dirs:
            try:
                ds = SequenceDataset(d, transform=self.transform)
                datasets.append(ds)
            except Exception as e:
                print(f"[WARN] Skipping {d}: {e}")

        if not datasets:
            raise RuntimeError(f"[ERROR] No valid datasets could be loaded under {self.base_dir}")

        self.concat_dataset = ConcatDataset(datasets)

    def _find_recording_dirs(self, base_dir: Path) -> List[Path]:
        candidates = []
        for path in base_dir.rglob("*"):
            if (path / "images").is_dir():
                candidates.append(path)
        return sorted(candidates)

    def __len__(self):
        return len(self.concat_dataset)

    def __getitem__(self, idx: int):
        return self.concat_dataset[idx]
