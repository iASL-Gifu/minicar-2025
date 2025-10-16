import torch
import cv2
import numpy as np
import random
from torchvision import transforms
from torchvision.transforms import v2

# --- 1. NumPy (OpenCV) ベースのクラス群 (ご提供のコード) ---
# (mode='numpy' で使用)

class CvCompose:
    def __init__(self, transforms):
        self.transforms = transforms
    def __call__(self, img_np):
        for t in self.transforms:
            img_np = t(img_np)
        return img_np

class CvResize:
    def __init__(self, height, width, interpolation=cv2.INTER_LINEAR):
        self.dsize = (width, height)
        self.interpolation = interpolation
    def __call__(self, img_np):
        return cv2.resize(img_np, self.dsize, interpolation=self.interpolation)

class CvColorJitter:
    def __init__(self, brightness=0.2, contrast=0.2, saturation=0.2, hue=0.0): # hue は互換性のため追加
        self.brightness = brightness
        self.contrast = contrast
        self.saturation = saturation
        # Note: hue の OpenCV 実装は重いため、ここでは 0.0 をデフォルトとし、
        # 0より大きい場合はHSV変換を行う
        self.hue = hue

    def __call__(self, img_np):
        beta = random.uniform(-self.brightness, self.brightness) * 255
        alpha = 1.0 + random.uniform(-self.contrast, self.contrast)
        img_np = cv2.convertScaleAbs(img_np, alpha=alpha, beta=beta)
        
        hsv = cv2.cvtColor(img_np, cv2.COLOR_RGB2HSV)
        
        # Saturation
        s_factor = 1.0 + random.uniform(-self.saturation, self.saturation)
        hsv[..., 1] = np.clip(hsv[..., 1].astype(np.float32) * s_factor, 0, 255).astype(np.uint8)
        
        # Hue
        if self.hue > 0:
            h_factor = random.uniform(-self.hue, self.hue) * 180 # OpenCVのHは 0-179
            hsv[..., 0] = (hsv[..., 0].astype(np.float32) + h_factor) % 180 # 180でラップアラウンド
            
        img_np = cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)
        return img_np

class CvRandomHorizontalFlip:
    def __init__(self, p=0.5):
        self.p = p
    def __call__(self, img_np):
        if random.random() < self.p:
            return cv2.flip(img_np, 1)
        return img_np

class CvToTensorNormalize:
    def __init__(self, mean, std):
        self.mean = np.array(mean, dtype=np.float32)
        self.std = np.array(std, dtype=np.float32)
    def __call__(self, img_np):
        img_np = img_np.astype(np.float32) / 255.0
        img_np = (img_np - self.mean) / self.std
        return torch.from_numpy(img_np.transpose(2, 0, 1))

# --- 2. 各モードのパイプラインを構築するヘルパー関数 ---

def get_numpy_transform(height, width, mean, std, is_train=True):
    """mode='numpy' 用のパイプラインを返す"""
    transform_list = []
    if is_train:
        transform_list.extend([
            CvColorJitter(brightness=0.2, contrast=0.2, saturation=0.2, hue=0.1),
            CvRandomHorizontalFlip(p=0.5),
        ])
    transform_list.extend([
        CvResize(height=height, width=width),
        CvToTensorNormalize(mean=mean, std=std)
    ])
    return CvCompose(transform_list)

def get_tensor_v2_transform(height, width, mean, std, is_train=True):
    """mode='tensor' (v2) 用のパイプラインを返す"""
    transform_list = []
    if is_train:
        transform_list.extend([
            v2.ColorJitter(brightness=0.2, contrast=0.2, saturation=0.2, hue=0.1),
            v2.RandomHorizontalFlip(p=0.5),
        ])
    transform_list.extend([
        v2.Resize((height, width), antialias=True),
        v2.ToDtype(torch.float32, scale=True), # ToTensor
        v2.Normalize(mean=mean, std=std),
    ])
    return v2.Compose(transform_list)

def get_pil_transform(height, width, mean, std, is_train=True):
    """mode='pil' (従来) 用のパイプラインを返す"""
    transform_list = [transforms.ToPILImage()]
    if is_train:
        transform_list.extend([
            transforms.ColorJitter(brightness=0.2, contrast=0.2, saturation=0.2, hue=0.1),
            transforms.RandomHorizontalFlip(p=0.5),
        ])
    transform_list.extend([
        transforms.Resize((height, width), antialias=True),
        transforms.ToTensor(),
        transforms.Normalize(mean=mean, std=std),
    ])
    return transforms.Compose(transform_list)

# --- 3. メインのTransformクラス (これを 2_train.py から呼ぶ) ---

class TrainTransform:
    """
    学習用の画像前処理クラス。
    modeに応じて内部パイプラインを切り替える。
    __call__ は常に NumPy (H, W, C) を受け取る。
    """
    def __init__(self, height=120, width=160, mode='numpy'):
        self.mode = mode
        mean = [0.5, 0.5, 0.5]
        std = [0.5, 0.5, 0.5]

        if mode == 'numpy':
            self.transform = get_numpy_transform(height, width, mean, std, is_train=True)
        elif mode == 'tensor':
            self.transform = get_tensor_v2_transform(height, width, mean, std, is_train=True)
        elif mode == 'pil':
            self.transform = get_pil_transform(height, width, mean, std, is_train=True)
        else:
            raise ValueError(f"無効なモードです: {mode}。'numpy', 'tensor', 'pil' のいずれかを選択してください。")

    def __call__(self, img_np):
        # datasetは常に NumPy (H, W, C) を渡す
        if self.mode == 'tensor':
            # 'tensor' (v2) モードのみ、(C, H, W) uint8 テンソルに変換してから渡す
            # .copy() は、NumPy配列が書き込み不可の場合があるため安全策として追加
            img_tensor = torch.from_numpy(img_np.copy()).permute(2, 0, 1)
            return self.transform(img_tensor)
        else:
            # 'numpy' と 'pil' モードは NumPy (H, W, C) をそのまま渡す
            return self.transform(img_np)


class TestTransform:
    """
    検証・テスト用の画像前処理クラス。
    modeに応じて内部パイプラインを切り替える。
    """
    def __init__(self, height=120, width=160, mode='numpy'):
        self.mode = mode
        mean = [0.5, 0.5, 0.5]
        std = [0.5, 0.5, 0.5]

        if mode == 'numpy':
            self.transform = get_numpy_transform(height, width, mean, std, is_train=False)
        elif mode == 'tensor':
            self.transform = get_tensor_v2_transform(height, width, mean, std, is_train=False)
        elif mode == 'pil':
            self.transform = get_pil_transform(height, width, mean, std, is_train=False)
        else:
            raise ValueError(f"無効なモードです: {mode}。'numpy', 'tensor', 'pil' のいずれかを選択してください。")

    def __call__(self, img_np):
        if self.mode == 'tensor':
            img_tensor = torch.from_numpy(img_np.copy()).permute(2, 0, 1)
            return self.transform(img_tensor)
        else:
            return self.transform(img_np)