import argparse
from pathlib import Path
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R

def transform_global_to_local(ref_pose_7d: np.ndarray, target_poses_7d: np.ndarray) -> np.ndarray:
    """
    ターゲットとなる複数のグローバル姿勢を、基準となる単一のグローバル姿勢のローカル座標系に変換する。

    Args:
        ref_pose_7d (np.ndarray): 基準となる姿勢 [x, y, z, qx, qy, qz, qw]
        target_poses_7d (np.ndarray): 変換したい姿勢群 (N, 7)

    Returns:
        np.ndarray: ローカル座標系に変換されたXY座標群 (N, 2)。
    """
    ref_position = ref_pose_7d[:3]
    ref_quat = ref_pose_7d[3:]
    target_positions = target_poses_7d[:, :3]

    translated = target_positions - ref_position
    inv_rotation = R.from_quat(ref_quat).inv()
    local_coords = inv_rotation.apply(translated)
    return local_coords[:, :2]

def draw_trajectory_on_bev(
    future_path_local: np.ndarray,
    past_odom_global: np.ndarray | None = None,
    canvas_size: tuple[int, int] = (400, 400),
    pixels_per_meter: int = 50,
    max_speed_ms: float = 2.0,
) -> np.ndarray:
    """
    過去と未来のローカルパスを鳥瞰図(BEV)キャンバスに描画する。

    Args:
        future_path_local (np.ndarray): 未来の軌跡データ (N, 3)。各点は (x, y, vx)。
        past_odom_global (np.ndarray | None): 過去のグローバルOdometry履歴 (M, 8)。
        canvas_size (tuple[int, int]): 生成するキャンバスのサイズ (幅, 高さ)。
        pixels_per_meter (int): 1メートルあたりのピクセル数。
        max_speed_ms (float): 色付けの基準となる最高速度 (m/s)。

    Returns:
        np.ndarray: 軌跡が描画されたBEV画像。
    """
    canvas = np.zeros((canvas_size[1], canvas_size[0], 3), dtype=np.uint8)
    robot_origin_u = canvas_size[0] // 2
    robot_origin_v = canvas_size[1] - 50

    # ロボット自体を三角形で描画
    robot_points = np.array([
        [robot_origin_u, robot_origin_v],
        [robot_origin_u - 10, robot_origin_v + 15],
        [robot_origin_u + 10, robot_origin_v + 15],
    ])
    cv2.fillPoly(canvas, [robot_points], (255, 255, 255))

    # --- 過去の軌跡を描画 (青色) ---
    if past_odom_global is not None and len(past_odom_global) > 1:
        current_pose_7d = past_odom_global[-1, :7]
        past_path_local = transform_global_to_local(current_pose_7d, past_odom_global[:, :7])
        
        for i, (x, y) in enumerate(past_path_local):
            u = int(robot_origin_u - y * pixels_per_meter)
            v = int(robot_origin_v - x * pixels_per_meter)
            color = (255, 128, 0)  # 明るい青 (BGR)

            if i > 0:
                prev_x, prev_y = past_path_local[i - 1]
                prev_u = int(robot_origin_u - prev_y * pixels_per_meter)
                prev_v = int(robot_origin_v - prev_x * pixels_per_meter)
                cv2.line(canvas, (prev_u, prev_v), (u, v), color, thickness=2)
            cv2.circle(canvas, (u, v), radius=3, color=color, thickness=-1)

    # --- 未来の軌跡を描画 (緑 -> 赤) ---
    for i, (x, y, vx) in enumerate(future_path_local):
        u = int(robot_origin_u - y * pixels_per_meter)
        v = int(robot_origin_v - x * pixels_per_meter)
        speed_ratio = min(vx / max_speed_ms, 1.0)
        color = (0, int(255 * (1 - speed_ratio)), int(255 * speed_ratio)) # B, G, R

        if i > 0:
            prev_x, prev_y, _ = future_path_local[i - 1]
            prev_u = int(robot_origin_u - prev_y * pixels_per_meter)
            prev_v = int(robot_origin_v - prev_x * pixels_per_meter)
            cv2.line(canvas, (prev_u, prev_v), (u, v), color, thickness=2)
        cv2.circle(canvas, (u, v), radius=3, color=color, thickness=-1)
        
    # --- 凡例を追加 ---
    cv2.putText(canvas, "Future", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    cv2.putText(canvas, "Past", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 128, 0), 2)

    return canvas


def visualize_and_save_dataset(dataset_dir: Path, output_save_dir: Path):
    """
    データセットを読み込み、画像とパスの鳥瞰図を結合して画像ファイルとして保存する。
    """
    dataset_dir = dataset_dir.resolve()
    output_save_dir = output_save_dir.resolve()
    output_save_dir.mkdir(parents=True, exist_ok=True)

    print(f"[INFO] Visualizing dataset from: {dataset_dir}")
    print(f"[INFO] Saving visualizations to: {output_save_dir}")

    # --- 必須ファイルとオプションファイルのパスを定義 ---
    future_paths_file = dataset_dir / "paths.npy"
    past_odoms_file = dataset_dir / "past_odoms.npy"
    images_dir = dataset_dir / "images"

    if not future_paths_file.exists() or not images_dir.is_dir():
        print(f"[ERROR] Required files ('paths.npy', 'images/') not found in {dataset_dir}")
        return

    future_paths = np.load(future_paths_file)
    past_odoms = np.load(past_odoms_file) if past_odoms_file.exists() else None
    
    num_samples = len(future_paths)
    print(f"[INFO] Found {num_samples} samples.")
    if past_odoms is not None:
        print("[INFO] Found 'past_odoms.npy'. Visualizing past trajectory as well.")

    for i in range(num_samples):
        image_path = images_dir / f"{i:06d}.png"
        if not image_path.exists(): continue

        image = cv2.imread(str(image_path))
        if image is None: continue

        future_path = future_paths[i]
        past_odom_history = past_odoms[i] if past_odoms is not None else None

        bev_canvas = draw_trajectory_on_bev(future_path, past_odom_history)

        # BEVと画像の高さを合わせるようにリサイズ
        bev_h, _, _ = bev_canvas.shape
        image_h, image_w, _ = image.shape
        scale_factor = bev_h / image_h
        resized_image = cv2.resize(image, (int(image_w * scale_factor), bev_h))

        combined_view = np.hstack((resized_image, bev_canvas))
        cv2.putText(combined_view, f"Sample {i:06d}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)

        save_path = output_save_dir / f"viz_{i:06d}.png"
        cv2.imwrite(str(save_path), combined_view)

        if (i + 1) % 100 == 0:
            print(f"[INFO] Processed {i + 1}/{num_samples} samples.")

    print(f"[INFO] Visualization complete. Total {i + 1} images saved to {output_save_dir}")


def main():
    parser = argparse.ArgumentParser(
        description="Visualize a trajectory dataset by combining images and Bird's-Eye-View paths.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "-d", "--dataset-dir",
        type=Path,
        required=True,
        help="Path to the processed dataset directory (e.g., ./dataset/my_bag_name)."
    )
    parser.add_argument(
        "-o", "--output-save-dir",
        type=Path,
        default=Path("./visualizations"),
        help="Directory to save the visualization images."
    )
    args = parser.parse_args()
    visualize_and_save_dataset(args.dataset_dir, args.output_save_dir)


if __name__ == "__main__":
    main()