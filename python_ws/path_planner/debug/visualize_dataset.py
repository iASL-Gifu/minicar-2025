import argparse
from pathlib import Path
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
import glob 

# --- この関数は提示されたものから変更ありません ---
def transform_global_to_local(ref_pose_7d: np.ndarray, target_poses_7d: np.ndarray) -> np.ndarray:
    """
    ターゲットとなる複数のグローバル姿勢を、基準となる単一のグローバル姿勢のローカル座標系に変換する。
    """
    ref_position = ref_pose_7d[:3]
    ref_quat = ref_pose_7d[3:]
    target_positions = target_poses_7d[:, :3]

    translated = target_positions - ref_position
    inv_rotation = R.from_quat(ref_quat).inv()
    local_coords = inv_rotation.apply(translated)
    return local_coords[:, :2]

# --- この関数は提示されたものから変更ありません ---
# (past_odom_global が 8次元ベクトル (M, 8) であることを想定していますが、
#  関数内部で [:7] とスライスしているので、データ抽出スクリプトの出力 (H, 8) と互換性があります)
def draw_trajectory_on_bev(
    future_path_local: np.ndarray,
    past_odom_global: np.ndarray | None = None,
    canvas_size: tuple[int, int] = (400, 400),
    pixels_per_meter: int = 50,
    max_speed_ms: float = 2.0,
) -> np.ndarray:
    """
    過去と未来のローカルパスを鳥瞰図(BEV)キャンバスに描画する。
    """
    canvas = np.zeros((canvas_size[1], canvas_size[0], 3), dtype=np.uint8)
    robot_origin_u = canvas_size[0] // 2
    robot_origin_v = canvas_size[1] - 50 # ロボットの原点を下から50pxに設定

    # ロボット自体を三角形で描画
    robot_points = np.array([
        [robot_origin_u, robot_origin_v],
        [robot_origin_u - 10, robot_origin_v + 15],
        [robot_origin_u + 10, robot_origin_v + 15],
    ])
    cv2.fillPoly(canvas, [robot_points], (255, 255, 255))

    # --- 過去の軌跡を描画 (青色) ---
    if past_odom_global is not None and len(past_odom_global) > 1:
        # 現在の姿勢（=過去の最後の姿勢）を基準にする
        current_pose_7d = past_odom_global[-1, :7] # (M, 8) -> (7,)
        # 過去の全姿勢をスライス
        past_poses_7d = past_odom_global[:, :7] # (M, 8) -> (M, 7)
        past_path_local = transform_global_to_local(current_pose_7d, past_poses_7d)
        
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
    # future_path_local は (N, 3) = (x, y, vx)
    for i, (x, y, vx) in enumerate(future_path_local):
        u = int(robot_origin_u - y * pixels_per_meter)
        v = int(robot_origin_v - x * pixels_per_meter)
        
        # 速度に応じて色を変化 (vx がマイナスの場合も考慮)
        speed_ratio = np.clip(vx / max_speed_ms, 0.0, 1.0)
        color = (0, int(255 * (1 - speed_ratio)), int(255 * speed_ratio)) # BGR: 低速(G) -> 高速(R)

        if i > 0:
            prev_x, prev_y, _ = future_path_local[i - 1]
            prev_u = int(robot_origin_u - prev_y * pixels_per_meter)
            prev_v = int(robot_origin_v - prev_x * pixels_per_meter)
            cv2.line(canvas, (prev_u, prev_v), (u, v), color, thickness=2)
        cv2.circle(canvas, (u, v), radius=3, color=color, thickness=-1)
        
    # --- 凡例を追加 ---
    cv2.putText(canvas, "Future Traj.", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    cv2.putText(canvas, "Past Odom", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 128, 0), 2)

    return canvas

# --- コマンド可視化パネル ---
def draw_commands_panel(
    future_cmds: np.ndarray,
    canvas_size: tuple[int, int] = (400, 400),
    max_steer_rad: float = 0.6, # 約34度 (仮)
    max_speed_ms: float = 2.0,
) -> np.ndarray:
    """
    未来の制御コマンド（ステアリング、速度）を可視化するパネルを描画する。
    """
    canvas = np.zeros((canvas_size[1], canvas_size[0], 3), dtype=np.uint8)
    
    # --- ステアリング (左半分) ---
    cv2.putText(canvas, "Future Steer", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    steer_center_u = canvas_size[0] // 4
    bar_width_max = (canvas_size[0] // 2) * 0.8
    bar_width = int(bar_width_max / len(future_cmds)) if len(future_cmds) > 0 else 1
    bar_width = max(1, bar_width) # 最小幅を1に
    
    bar_spacing = 0
    if len(future_cmds) > 1:
        bar_spacing = int(( (canvas_size[0] // 2) * 0.2 ) / (len(future_cmds) - 1))
        
    start_offset = (canvas_size[0] // 2 - (bar_width * len(future_cmds) + bar_spacing * (len(future_cmds) -1))) // 2


    for i, (steer, _) in enumerate(future_cmds):
        u_start = start_offset + (bar_width + bar_spacing) * i
        u_end = u_start + bar_width
        
        steer_ratio = np.clip(steer / max_steer_rad, -1.0, 1.0)
        v_center = canvas_size[1] // 2
        bar_height = int(steer_ratio * (canvas_size[1] / 2 * 0.8))
        
        color = (0, 200, 0) if steer_ratio <= 0 else (0, 0, 200) # (BGR) 緑:左, 赤:右
        cv2.rectangle(canvas, (u_start, v_center), (u_end, v_center - bar_height), color, -1)

    cv2.line(canvas, (0, canvas_size[1] // 2), (canvas_size[0] // 2, canvas_size[1] // 2), (255, 255, 255), 1)

    # --- 速度 (右半分) ---
    cv2.putText(canvas, "Future Speed", (canvas_size[0] // 2 + 10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    speed_base_v = canvas_size[1] - 20
    start_offset_speed = (canvas_size[0] // 2) + start_offset

    for i, (_, speed) in enumerate(future_cmds):
        u_start = start_offset_speed + (bar_width + bar_spacing) * i
        u_end = u_start + bar_width
        
        speed_ratio = np.clip(speed / max_speed_ms, 0.0, 1.0)
        bar_height = int(speed_ratio * (canvas_size[1] * 0.8))
        
        color = (0, 255, 0) # 緑
        cv2.rectangle(canvas, (u_start, speed_base_v), (u_end, speed_base_v - bar_height), color, -1)
        
    cv2.line(canvas, (canvas_size[0] // 2, speed_base_v), (canvas_size[0], speed_base_v), (255, 255, 255), 1)

    return canvas

# --- データ読み込みロジックを変更 ---
def visualize_and_save_dataset(dataset_dir: Path, output_save_dir: Path):
    """
    データセットを読み込み、画像、軌跡BEV、コマンドパネルを結合して保存する。
    """
    dataset_dir = dataset_dir.resolve()
    output_save_dir = output_save_dir.resolve()
    output_save_dir.mkdir(parents=True, exist_ok=True)

    print(f"[INFO] Visualizing dataset from: {dataset_dir}")
    print(f"[INFO] Saving visualizations to: {output_save_dir}")

    images_dir = dataset_dir / "images"
    if not images_dir.is_dir():
        print(f"[ERROR] 'images/' directory not found in {dataset_dir}")
        return

    # 'images' ディレクトリ内のPNGファイルを基準にループ
    image_paths = sorted(glob.glob(str(images_dir / "*.png")))
    num_samples = len(image_paths)
    print(f"[INFO] Found {num_samples} image samples.")
    
    if num_samples == 0:
        print("[WARN] No images found. Exiting.")
        return

    for i, image_path_str in enumerate(image_paths):
        image_path = Path(image_path_str)
        sample_stem = image_path.stem # 例: "000000"
        
        # 対応する .npy ファイルのパスを構築
        past_odom_file = dataset_dir / f"past_odoms_{sample_stem}.npy"
        future_traj_file = dataset_dir / f"future_trajectory_{sample_stem}.npy"
        future_cmds_file = dataset_dir / f"future_cmds_{sample_stem}.npy"

        if not (past_odom_file.exists() and future_traj_file.exists() and future_cmds_file.exists()):
            print(f"[WARN] Skipping sample {sample_stem}: Missing one or more .npy files.")
            continue

        image = cv2.imread(str(image_path))
        if image is None: 
            print(f"[WARN] Skipping sample {sample_stem}: Failed to load image.")
            continue

        try:
            past_odom_history = np.load(past_odom_file)     # (H, 8)
            future_trajectory = np.load(future_traj_file) # (F, 3)
            future_cmds = np.load(future_cmds_file)       # (F, 2)
        except Exception as e:
            print(f"[ERROR] Failed to load .npy for sample {sample_stem}: {e}")
            continue

        # --- 各パネルを描画 ---
        # 1. BEVパネル (過去 + 未来軌跡)
        # (past_odom は 8次元, future_traj は 3次元(x,y,vx) を想定)
        bev_canvas = draw_trajectory_on_bev(future_trajectory, past_odom_history)
        
        # 2. コマンドパネル (未来コマンド)
        cmd_canvas = draw_commands_panel(future_cmds)

        # BEVと画像の高さを合わせるようにリサイズ
        bev_h, bev_w, _ = bev_canvas.shape
        image_h, image_w, _ = image.shape
        scale_factor = bev_h / image_h
        resized_image = cv2.resize(image, (int(image_w * scale_factor), bev_h))
        
        # コマンドパネルも高さを合わせる
        resized_cmd_canvas = cv2.resize(cmd_canvas, (bev_w, bev_h))

        # 3つの画像を水平に結合
        combined_view = np.hstack((resized_image, bev_canvas, resized_cmd_canvas))
        cv2.putText(combined_view, f"Sample {sample_stem}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)

        save_path = output_save_dir / f"viz_{sample_stem}.png"
        cv2.imwrite(str(save_path), combined_view)

        if (i + 1) % 100 == 0:
            print(f"[INFO] Processed {i + 1}/{num_samples} samples.")

    print(f"[INFO] Visualization complete. Images saved to {output_save_dir}")


def main():
    parser = argparse.ArgumentParser(
        description="Visualize a trajectory dataset by combining images, BEV paths, and command plots.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "-d", "--datadir",
        type=Path,
        required=True,
        help="Path to the processed dataset directory (e.g., ./dataset/my_bag_name) "
             "containing 'images/', 'past_odoms_*.npy', 'future_trajectory_*.npy', and 'future_cmds_*.npy'."
    )
    parser.add_argument(
        "-o", "--output",
        type=Path,
        default=Path("./outputs/visualizations"),
        help="Directory to save the visualization images."
    )
    args = parser.parse_args()
    visualize_and_save_dataset(args.datadir, args.output)


if __name__ == "__main__":
    main()