import torch
from torch.utils.data import Dataset
from pathlib import Path
import numpy as np
import cv2

class RecordingSequenceDataset(Dataset):
    """
    個別の記録データから抽出した画像シーケンスと制御量を扱うためのDatasetクラス。

    Args:
        root_dir (str): 抽出されたデータが格納されているルートディレクトリ。
        sequence_length (int): 1サンプルとして扱うシーケンスの長さ。
        transform (callable, optional): 画像に適用する変換処理。
    """
    def __init__(self, root_dir, sequence_length=1, transform=None):
        self.root_dir = Path(root_dir)
        self.sequence_length = sequence_length
        self.transform = transform
        
        self.recordings_data = []
        self.sample_indices = []

        # 各記録データ（ディレクトリ）を探索
        recording_dirs = sorted([p for p in self.root_dir.iterdir() if p.is_dir()])

        for recording_idx, recording_path in enumerate(recording_dirs):
            image_dir = recording_path / 'images'
            steers_path = recording_path / 'steers.npy'
            speeds_path = recording_path / 'speeds.npy'

            if not (image_dir.exists() and steers_path.exists() and speeds_path.exists()):
                continue

            image_paths = sorted(image_dir.glob('*.png'))
            steers = np.load(steers_path)
            speeds = np.load(speeds_path)
            
            if not (len(image_paths) == len(steers) == len(speeds)):
                print(f"[WARN] Skipping {recording_path.name}: data length mismatch.")
                continue

            self.recordings_data.append({
                'images': image_paths,
                'steers': steers,
                'speeds': speeds,
            })
            
            num_frames = len(image_paths)
            if num_frames >= self.sequence_length:
                for i in range(num_frames - self.sequence_length + 1):
                    self.sample_indices.append((recording_idx, i))

    def __len__(self):
        return len(self.sample_indices)

    def __getitem__(self, idx):
        recording_idx, start_frame = self.sample_indices[idx]
        end_frame = start_frame + self.sequence_length
        
        recording_data = self.recordings_data[recording_idx]
        
        image_paths_seq = recording_data['images'][start_frame:end_frame]
        steers_seq = recording_data['steers'][start_frame:end_frame]
        speeds_seq = recording_data['speeds'][start_frame:end_frame]
        
        images = []
        for img_path in image_paths_seq:
            image = cv2.imread(str(img_path))
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            
            if self.transform:
                image = self.transform(image)
            images.append(image)
        
        image_tensor_seq = torch.stack(images)
        labels_tensor_seq = torch.tensor(np.stack([steers_seq, speeds_seq], axis=-1), dtype=torch.float32)
        
        if self.sequence_length == 1:
            image_tensor_seq = image_tensor_seq.squeeze(0)
            labels_tensor_seq = labels_tensor_seq.squeeze(0)

        return image_tensor_seq, labels_tensor_seq
