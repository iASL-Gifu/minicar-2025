from torchvision import transforms

class TrainTransform:
    """学習用の画像前処理クラス。データ拡張を含む。"""
    def __init__(self, height=120, width=160):
        self.transform = transforms.Compose([
            transforms.ColorJitter(brightness=0.2, contrast=0.2, saturation=0.2, hue=0.1),
            transforms.RandomHorizontalFlip(p=0.5),
            transforms.Resize((height, width), antialias=True),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.5, 0.5, 0.5], std=[0.5, 0.5, 0.5]),
        ])

    def __call__(self, image):
        return self.transform(image)


class TestTransform:
    """検証・テスト用の画像前処理クラス。データ拡張を含まない。"""
    def __init__(self, height=120, width=160):
        self.transform = transforms.Compose([
            transforms.Resize((height, width), antialias=True),
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.5, 0.5, 0.5], std=[0.5, 0.5, 0.5]),
        ])

    def __call__(self, image):
        return self.transform(image)