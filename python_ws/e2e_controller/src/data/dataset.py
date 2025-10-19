import glob
from pathlib import Path
from typing import Dict, List, Optional, Union
import torch
from torch.utils.data import Dataset, ConcatDataset
import numpy as np
import cv2


# =========================================================
# 1. SequenceDataset: 単一の sequence ディレクトリを対象とする
# =========================================================
class SequenceDataset(Dataset):
    def __init__(self, seq_dir: str, transform=None, seq_len: int = 10):
        """
        単一のシーケンス（例: rosbagから変換された1フォルダ）を読み込む
        odom (odoms.npy) は任意。存在しない場合はダミーデータ(0埋め)を返す。
        """
        self.seq_dir = Path(seq_dir)
        self.transform = transform
        self.seq_len = seq_len

        # --- 各データのパスを取得 ---
        self.image_files = sorted(glob.glob(str(self.seq_dir / "images" / "*.png")))
        self.steer_file = self.seq_dir / "steers.npy"
        self.speed_file = self.seq_dir / "speeds.npy"
        self.odom_file = self.seq_dir / "odoms.npy"

        # --- 必須ファイルの存在チェック ---
        if not self.image_files:
             raise FileNotFoundError(f"No images found in {self.seq_dir / 'images'}")
        if not self.steer_file.exists():
            raise FileNotFoundError(f"steers.npy not found in {self.seq_dir}")
        if not self.speed_file.exists():
            raise FileNotFoundError(f"speeds.npy not found in {self.seq_dir}")

        # --- ラベル類を読み込み ---
        self.steers = np.load(self.steer_file)
        self.speeds = np.load(self.speed_file)
        
        # --- Odom の読み込み (任意) ---
        self.has_odom = self.odom_file.exists()
        if self.has_odom:
            self.odoms = np.load(self.odom_file)
            # print(f"[SequenceDataset INFO] Loaded 'odoms.npy' from {self.seq_dir}")
        else:
            # odom がない場合、ダミーデータを作成 (形状は (N, 7) と仮定)
            print(f"[SequenceDataset WARN] No 'odoms.npy' in {self.seq_dir}. Filling with zeros.")
            num_frames = len(self.image_files)
            # (pos 3 + quat 4 = 7)
            DUMMY_ODOM_DIM = 7 
            self.odoms = np.zeros((num_frames, DUMMY_ODOM_DIM), dtype=np.float32)

        # ---  アサーション ---
        assert len(self.image_files) == len(self.steers) == len(self.speeds) == len(self.odoms), \
            f"Data length mismatch in {seq_dir}: " \
            f"Images({len(self.image_files)}), Steers({len(self.steers)}), " \
            f"Speeds({len(self.speeds)}), Odoms({len(self.odoms)})"

    def __len__(self):
        #  利用可能なシーケンスの開始インデックス数
        num_frames = len(self.image_files)
        if num_frames < self.seq_len:
            return 0
        return num_frames - self.seq_len + 1

    def __getitem__(self, idx: int) -> Dict[str, Union[torch.Tensor, np.ndarray]]:
        # --- 画像シーケンスを読み込み ---
        img_seq = []
        for i in range(self.seq_len):
            img_path = self.image_files[idx + i]
            img = cv2.imread(img_path)
            #  画像読み込み失敗チェック
            if img is None:
                raise IOError(f"Failed to load image: {img_path}")
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) # (H, W, C) uint8 NumPy
            img_seq.append(img)

        # (seq_len, H, W, C) uint8 NumPy 配列
        image_seq_numpy = np.stack(img_seq) 
        
        steers_tensor_seq = torch.tensor(self.steers[idx:idx + self.seq_len], dtype=torch.float32)
        speeds_tensor_seq = torch.tensor(self.speeds[idx:idx + self.seq_len], dtype=torch.float32)
        odoms_tensor_seq = torch.tensor(self.odoms[idx:idx + self.seq_len], dtype=torch.float32)

        sample = {
            'image': image_seq_numpy, # ★ NumPy 配列 (seq_len, H, W, C) のまま渡す
            'steer': steers_tensor_seq,
            'speed': speeds_tensor_seq,
            'odom': odoms_tensor_seq,
            'has_odom': torch.tensor(self.has_odom, dtype=torch.bool) # [NEW] odom が本物かダミーかのフラグ
        }

        if self.transform:
            sample = self.transform(sample) # ★ Transform が NumPy -> Tensor 変換を行う

        return sample


# =========================================================
# 2. MultiSequenceDataset: 複数シーケンスを統合 + 部分選択機能付き
# =========================================================
class MultiSequenceDataset(Dataset):
    def __init__(
        self,
        base_dir: str,
        transform=None,
        seq_len: int = 10,
        select_sequences: Optional[Union[List[int], range]] = None,
    ):
        """
        base_dir 以下を再帰的に探索し、各 sequence ディレクトリをまとめる。
        select_sequences を指定すると、その範囲だけを利用可能。
        """
        self.base_dir = Path(base_dir)
        self.transform = transform
        self.seq_len = seq_len

        # --- 再帰的探索 ---
        self.seq_dirs = self._find_sequence_dirs(self.base_dir)
        if not self.seq_dirs:
            raise RuntimeError(f"No valid sequence directories found under {base_dir}")

        print(f"[MultiSequenceDataset] Found {len(self.seq_dirs)} sequences under {base_dir}")

        # --- 特定シーケンスのみに限定 ---
        if select_sequences is not None:
            selected_indices = list(select_sequences)
            self.seq_dirs = [self.seq_dirs[i] for i in selected_indices if i < len(self.seq_dirs)]
            print(f"[MultiSequenceDataset] Selected {len(self.seq_dirs)} sequences (indices={selected_indices})")

        # --- 各シーケンスを構築 ---
        self.datasets: List[SequenceDataset] = [
            SequenceDataset(d, transform=self.transform, seq_len=self.seq_len)
            for d in self.seq_dirs
        ]

        # --- ConcatDataset で統合 ---
        self.concat_dataset = ConcatDataset(self.datasets)

    def _find_sequence_dirs(self, base_dir: Path) -> List[Path]:
        """
        [MODIFIED] 再帰的に探索し、
        'images' (dir), 'steers.npy', 'speeds.npy' があるディレクトリをsequenceと認定
        """
        seq_dirs = []
        # 'steers.npy' を起点に親ディレクトリをチェックする方が効率的
        for steer_file in base_dir.rglob('steers.npy'):
            path = steer_file.parent
            # 'images' フォルダがあり、'speeds.npy' ファイルがあればOK
            # odoms.npy の有無は問わない
            if (path / 'images').is_dir() and (path / 'speeds.npy').exists():
                seq_dirs.append(path)
        
        seq_dirs.sort()
        return seq_dirs

    def list_sequences(self) -> List[str]:
        """全シーケンスのパスを一覧表示"""
        return [str(d) for d in self.seq_dirs]

    def get_sequence(self, idx: int) -> SequenceDataset:
        """個別の SequenceDataset を取得"""
        return self.datasets[idx]

    def __len__(self):
        return len(self.concat_dataset)

    def __getitem__(self, idx):
        return self.concat_dataset[idx]