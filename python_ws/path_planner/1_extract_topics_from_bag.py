from pathlib import Path
import numpy as np
from rosbags.highlevel import AnyReader
import cv2
from scipy.spatial.transform import Rotation as R # scipy をインポート

def transform_global_to_local(ref_pose_7d: np.ndarray, target_poses_7d: np.ndarray) -> np.ndarray:
    """
    ターゲットとなる複数のグローバル姿勢を、基準となる単一のグローバル姿勢のローカル座標系に変換する。
    (x, y, z) -> (x, y) の2D座標のみ返す。

    Args:
        ref_pose_7d (np.ndarray): 基準となる姿勢 [x, y, z, qx, qy, qz, qw] (7,)
        target_poses_7d (np.ndarray): 変換したい姿勢群 (N, 7)

    Returns:
        np.ndarray: ローカル座標系に変換されたXY座標群 (N, 2)。
    """
    ref_position = ref_pose_7d[:3]
    ref_quat = ref_pose_7d[3:]
    target_positions = target_poses_7d[:, :3]

    # 基準位置を原点に
    translated = target_positions - ref_position
    # 基準の向きがローカルの+X軸になるように回転
    inv_rotation = R.from_quat(ref_quat).inv()
    local_coords = inv_rotation.apply(translated)
    
    # ローカル座標の (x, y) のみ返す
    return local_coords[:, :2]

def extract_temporal_samples(
    bag_path,
    output_dir,
    image_topic,
    cmd_topic,
    odom_topic,
    history_len=10,
    history_step=2,
    future_len=30,  # デフォルトを30に変更
    future_step=3
):
    bag_path = Path(bag_path).expanduser().resolve()
    bag_name = bag_path.name
    out_dir = Path(output_dir) / bag_name
    out_dir.mkdir(parents=True, exist_ok=True)

    image_data, image_times = [], []
    cmd_data, cmd_times = [], []
    odom_data, odom_times = [], [] # (x, y, z, qx, qy, qz, qw, vx) の8次元

    with AnyReader([bag_path]) as reader:
        connections = [
            c for c in reader.connections if c.topic in [image_topic, cmd_topic, odom_topic]
        ]

        for conn, timestamp, raw in reader.messages(connections=connections):
            msg = reader.deserialize(raw, conn.msgtype)

            # --- 画像 ---
            if conn.topic == image_topic and conn.msgtype == "sensor_msgs/msg/Image":
                encoding = msg.encoding
                if encoding in ["bgr8", "rgb8"]:
                    shape = (msg.height, msg.width, 3)
                else:
                    continue
                image_np = np.frombuffer(msg.data, dtype=np.uint8).reshape(shape)
                if encoding == "rgb8":
                    image_np = cv2.cvtColor(image_np, cv2.COLOR_RGB2BGR)
                image_data.append(image_np)
                image_times.append(timestamp)

            # --- 制御 ---
            elif conn.topic == cmd_topic and conn.msgtype == "ackermann_msgs/msg/AckermannDriveStamped":
                cmd_data.append(np.array([msg.drive.steering_angle, msg.drive.speed], dtype=np.float32))
                cmd_times.append(timestamp)

            # --- オドメトリ ---
            elif conn.topic == odom_topic and conn.msgtype == "nav_msgs/msg/Odometry":
                pose = msg.pose.pose
                pos = np.array([pose.position.x, pose.position.y, pose.position.z])
                ori = np.array([
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w
                ])
                # vx (twist.twist.linear.x) を取得
                vx = msg.twist.twist.linear.x
                
                # pos(3), ori(4), vx(1) を結合して 8次元ベクトルにする
                odom_vec = np.concatenate([pos, ori, np.array([vx])]).astype(np.float32)
                odom_data.append(odom_vec)
                odom_times.append(timestamp)

    # --- 同期 ---
    if not (image_data and cmd_data and odom_data):
        print(f"[WARN] {bag_name}: insufficient data, skipped.")
        return

    image_times = np.array(image_times)
    cmd_data, cmd_times = np.array(cmd_data), np.array(cmd_times)
    odom_data, odom_times = np.array(odom_data), np.array(odom_times)

    synced = []
    for i, itime in enumerate(image_times):
        idx_cmd = np.argmin(np.abs(cmd_times - itime))
        idx_odom = np.argmin(np.abs(odom_times - itime))
        synced.append((i, idx_cmd, idx_odom)) # (img_idx, cmd_idx, odom_idx)

    # --- サンプル構築 ---
    samples = []
    # ループ範囲の確保
    start_idx = history_len * history_step
    end_idx = len(synced) - future_len * future_step
    
    if start_idx >= end_idx:
        print(f"[WARN] {bag_name}: Not enough synced data to create samples.")
        return

    for i in range(start_idx, end_idx):
        
        # --- 過去オドメトリ (H, 8) ---
        past_odoms_raw = []
        for h in range(history_len):
            # i, i-step, i-2*step ...
            _, _, idx_odom = synced[i - h * history_step]
            past_odoms_raw.append(odom_data[idx_odom])
        # 時系列順 (古い -> 現在の手前) に並べ替え
        past_odoms = np.stack(past_odoms_raw[::-1]) 

        # --- 未来データ (コマンドとオドメトリ) ---
        future_cmds_raw = []
        future_odoms_global_raw = []
        for f in range(1, future_len + 1):
            # i+step, i+2*step ...
            _, idx_cmd, idx_odom = synced[i + f * future_step]
            future_cmds_raw.append(cmd_data[idx_cmd])
            future_odoms_global_raw.append(odom_data[idx_odom])
            
        future_cmds = np.stack(future_cmds_raw) # (F, 2)
        future_odoms_global = np.stack(future_odoms_global_raw) # (F, 8)

        # --- 現在画像 と 現在オドメトリ ---
        img_idx, _, current_odom_idx = synced[i]
        img = image_data[img_idx]
        current_odom_8d = odom_data[current_odom_idx] # (8,)

        # 未来のローカル軌跡 (x, y, vx) を作成
        current_pose_7d = current_odom_8d[:7] # 現在の姿勢 (7,)
        future_poses_7d = future_odoms_global[:, :7] # 未来の姿勢 (F, 7)
        future_vx = future_odoms_global[:, 7] # 未来の速度 (F,)

        # 未来のグローバル姿勢 (F, 7) をローカルの (x, y) 座標 (F, 2) に変換
        future_path_local_xy = transform_global_to_local(current_pose_7d, future_poses_7d)
        
        # (F, 2) と (F, 1) を結合して (F, 3) の軌跡データにする
        future_trajectory = np.hstack((
            future_path_local_xy, 
            future_vx.reshape(-1, 1) # (F,) -> (F, 1) に変形
        )).astype(np.float32)

        # (画像, 過去odom(H,8), 未来cmd(F,2), 未来軌跡(F,3))
        samples.append((img, past_odoms, future_cmds, future_trajectory))

    # --- 保存 ---
    img_dir = out_dir / "images"
    img_dir.mkdir(exist_ok=True)
    
    if not samples:
        print(f"[WARN] {bag_name}: No samples were created (check lengths and steps).")
        return

    for i, (img, past_odoms, future_cmds, future_trajectory) in enumerate(samples):
        img_path = str(img_dir / f"{i:06d}.png")
        past_odom_path = str(out_dir / f"past_odoms_{i:06d}.npy")
        future_cmd_path = str(out_dir / f"future_cmds_{i:06d}.npy")
        future_traj_path = str(out_dir / f"future_trajectory_{i:06d}.npy") 

        cv2.imwrite(img_path, img)
        np.save(past_odom_path, past_odoms)           # (H, 8)
        np.save(future_cmd_path, future_cmds)         # (F, 2)
        np.save(future_traj_path, future_trajectory)  # (F, 3) <- (x, y, vx)

    print(f"[SAVE] {bag_name}: {len(samples)} samples saved "
          f"(H{history_len}xS{history_step}, F{future_len}xS{future_step})")


def main():
    import argparse

    parser = argparse.ArgumentParser(description="Extract temporal image-odom-cmd samples from ROS2 bag")
    parser.add_argument("--bags_dir", required=True, help="Path to directory containing rosbag folders")
    parser.add_argument("--outdir", required=True, help="Output directory path")
    parser.add_argument("--image_topic", default="/realsense2_camera/color/image_raw", help="Image topic name")
    parser.add_argument("--cmd_topic", default="/jetracer/cmd_drive", help="AckermannDriveStamped topic")
    parser.add_argument("--odom_topic", default="/visual_slam/tracking/odometry", help="Odometry topic")
    
    # デフォルト値を変更
    parser.add_argument("--history_len", type=int, default=10, help="Number of past odom steps")
    parser.add_argument("--history_step", type=int, default=1, help="Step size for past odom")
    parser.add_argument("--future_len", type=int, default=30, help="Number of future cmd/trajectory steps")
    parser.add_argument("--future_step", type=int, default=1, help="Step size for future cmd/trajectory")
    
    args = parser.parse_args()

    bags_dir = Path(args.bags_dir).expanduser().resolve()
    # Bagファイル自体 (db3) や "metadata.yaml" を直接指定するのではなく、
    # それらを含むディレクトリを指定することを想定 (rosbag record -o my_bag_dir の形式)
    bag_dirs = [p for p in bags_dir.iterdir() if p.is_dir() and (p / "metadata.yaml").exists()]

    if not bag_dirs:
        print(f"[ERROR] No valid rosbag directories found in {bags_dir}.")
        print("Usage: --bags_dir should point to a directory *containing* bag folders (e.g., 'my_bag_01', 'my_bag_02').")
        
        # もし bags_dir 自体が bag フォルダだった場合も考慮
        if (bags_dir / "metadata.yaml").exists():
            print(f"[INFO] Treating {bags_dir} as a single bag directory.")
            bag_dirs = [bags_dir]
        else:
            return

    print(f"[INFO] Found {len(bag_dirs)} rosbag directories.")
    for bag_path in sorted(bag_dirs):
        print(f"--- Processing {bag_path.name} ---")
        extract_temporal_samples(
            bag_path,
            args.outdir,
            args.image_topic,
            args.cmd_topic,
            args.odom_topic,
            args.history_len,
            args.history_step,
            args.future_len,
            args.future_step,
        )


if __name__ == "__main__":
    main()