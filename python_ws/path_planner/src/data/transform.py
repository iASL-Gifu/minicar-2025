import torch
import cv2
import numpy as np
import random
from typing import Dict

# -----------------------------
# 1. NumPyベースのTransform (修正版)
# -----------------------------
class CvCompose:
    """サンプル辞書を受け取るように変更"""
    def __init__(self, transforms):
        self.transforms = transforms
    def __call__(self, sample: Dict) -> Dict:
        for t in self.transforms:
            sample = t(sample)
        return sample

class CvResize:
    """サンプル辞書の 'image' のみをリサイズ"""
    def __init__(self, height, width, interpolation=cv2.INTER_LINEAR):
        self.dsize = (width, height)
        self.interpolation = interpolation
    def __call__(self, sample: Dict) -> Dict:
        sample['image'] = cv2.resize(sample['image'], self.dsize, interpolation=self.interpolation)
        return sample

class CvColorJitter:
    """サンプル辞書の 'image' のみにJitterを適用"""
    def __init__(self, brightness=0.2, contrast=0.2, saturation=0.2, hue=0.0):
        self.brightness = brightness
        self.contrast = contrast
        self.saturation = saturation
        self.hue = hue

    def __call__(self, sample: Dict) -> Dict:
        img_np = sample['image']
        
        beta = random.uniform(-self.brightness, self.brightness) * 255
        alpha = 1.0 + random.uniform(-self.contrast, self.contrast)
        img_np = cv2.convertScaleAbs(img_np, alpha=alpha, beta=beta)
        
        hsv = cv2.cvtColor(img_np, cv2.COLOR_RGB2HSV)
        s_factor = 1.0 + random.uniform(-self.saturation, self.saturation)
        hsv[..., 1] = np.clip(hsv[..., 1].astype(np.float32) * s_factor, 0, 255).astype(np.uint8)
        if self.hue > 0:
            h_factor = random.uniform(-self.hue, self.hue) * 180
            hsv[..., 0] = (hsv[..., 0].astype(np.float32) + h_factor) % 180
        img_np = cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)
        
        sample['image'] = img_np
        return sample

class CvRandomHorizontalFlip:
    """【重要】画像、軌跡、コマンド、オドメトリを協調して反転"""
    def __init__(self, p=0.5):
        self.p = p
    def __call__(self, sample: Dict) -> Dict:
        if random.random() < self.p:
            # 1. 画像を反転
            sample['image'] = cv2.flip(sample['image'], 1)
            
            # 2. 過去オドメトリ (H, 8) [x, y, z, qx, qy, qz, qw, vx]
            #    y座標, qy, qz を反転
            sample['past_odoms'][:, 1] *= -1.0 # y
            sample['past_odoms'][:, 4] *= -1.0 # qy
            sample['past_odoms'][:, 5] *= -1.0 # qz
            
            # 3. 未来軌跡 (F, 3) [x, y, vx]
            #    y座標を反転
            sample['future_path'][:, 1] *= -1.0 # y
            
            # 4. 未来コマンド (F, 2) [steer, speed]
            #    steerを反転
            sample['future_cmd'][:, 0] *= -1.0 # steer
            
        return sample

class CvToTensorNormalize:
    """辞書全体をテンソルに変換し、画像を正規化"""
    def __init__(self, mean, std):
        self.mean = np.array(mean, dtype=np.float32)
        self.std = np.array(std, dtype=np.float32)
        
    def __call__(self, sample: Dict) -> Dict[str, torch.Tensor]:
        # 1. 画像を正規化
        img_np = sample['image'].astype(np.float32) / 255.0
        img_np = (img_np - self.mean) / self.std
        
        # 2. 辞書全体をテンソルに変換
        output_tensor_dict = {
            'image': torch.from_numpy(img_np.transpose(2, 0, 1)), # (C, H, W)
            'past_odoms': torch.tensor(sample['past_odoms'], dtype=torch.float32),
            'future_path': torch.tensor(sample['future_path'], dtype=torch.float32),
            'future_cmd': torch.tensor(sample['future_cmd'], dtype=torch.float32)
        }
        
        return output_tensor_dict


# -----------------------------
# 2. Transform ヘルパー (修正版)
# -----------------------------
def get_numpy_transform(height, width, mean, std, is_train=True):
    """新しいCv*クラス群を使ってパイプラインを構築"""
    transform_list = []
    if is_train:
        transform_list.extend([
            CvColorJitter(brightness=0.2, contrast=0.2, saturation=0.2, hue=0.1),
            CvRandomHorizontalFlip(p=0.5) # 協調反転
        ])
    transform_list.extend([
        CvResize(height, width),
        CvToTensorNormalize(mean, std) # 最後にテンソル化
    ])
    return CvCompose(transform_list)


# -----------------------------
# 3. Dataset用 Transformクラス (修正版)
# -----------------------------
class TrainTransform:
    """学習用 (NumPy-based pipeline)"""
    def __init__(self, height=120, width=160):
        mean = [0.5, 0.5, 0.5]
        std = [0.5, 0.5, 0.5]
        # get_numpy_transform が CvCompose インスタンスを返す
        self.transform = get_numpy_transform(height, width, mean, std, is_train=True)

    def __call__(self, sample_dict_np: Dict) -> Dict[str, torch.Tensor]:
        """
        Datasetから渡されたNumPy辞書を
        Transformパイプラインに通す。
        """
        return self.transform(sample_dict_np)


class TestTransform:
    """検証・テスト用 (NumPy-based pipeline)"""
    def __init__(self, height=120, width=160):
        mean = [0.5, 0.5, 0.5]
        std = [0.5, 0.5, 0.5]
        self.transform = get_numpy_transform(height, width, mean, std, is_train=False)

    def __call__(self, sample_dict_np: Dict) -> Dict[str, torch.Tensor]:
        return self.transform(sample_dict_np)