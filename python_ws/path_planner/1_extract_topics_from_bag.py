import argparse
import multiprocessing  # 並列処理
import os
from pathlib import Path

import cv2
import numpy as np
from rosbags.highlevel import AnyReader
from scipy.spatial.transform import Rotation as R  # scipy をインポート


def transform_global_to_local(ref_pose_7d: np.ndarray, target_poses_7d: np.ndarray) -> np.ndarray:
    """
    ターゲットとなる複数のグローバル姿勢を、基準となる単一のグローバル姿勢のローカル座標系に変換する。
    (x, y, z) -> (x, y) の2D座標のみ返す。
    """
    ref_position = ref_pose_7d[:3]
    ref_quat = ref_pose_7d[3:]
    target_positions = target_poses_7d[:, :3]

    translated = target_positions - ref_position
    inv_rotation = R.from_quat(ref_quat).inv()
    local_coords = inv_rotation.apply(translated)
    return local_coords[:, :2]


def extract_temporal_samples(
    bag_path,
    output_dir,
    image_topic,
    cmd_topic,
    odom_topic,
    history_len=10,
    history_step=2,
    future_len=30,
    future_step=3
):
    """
    単一のROSバッグディレクトリから時系列サンプルを抽出するワーカ関数。
    """
    pid = os.getpid()
    bag_path = Path(bag_path).expanduser().resolve()
    bag_name = bag_path.name
    out_dir = Path(output_dir) / bag_name
    out_dir.mkdir(parents=True, exist_ok=True)

    image_data, image_times = [], []
    cmd_data, cmd_times = [], []
    odom_data, odom_times = [], []  # (x, y, z, qx, qy, qz, qw, speed)

    try:
        with AnyReader([bag_path]) as reader:
            connections = [c for c in reader.connections if c.topic in [image_topic, cmd_topic, odom_topic]]

            for conn, timestamp, raw in reader.messages(connections=connections):
                msg = reader.deserialize(raw, conn.msgtype)

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

                elif conn.topic == cmd_topic and conn.msgtype == "ackermann_msgs/msg/AckermannDriveStamped":
                    cmd_data.append(np.array([msg.drive.steering_angle, msg.drive.speed], dtype=np.float32))
                    cmd_times.append(timestamp)

                elif conn.topic == odom_topic and conn.msgtype == "nav_msgs/msg/Odometry":
                    pose = msg.pose.pose
                    pos = np.array([pose.position.x, pose.position.y, pose.position.z])
                    ori = np.array([
                        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w
                    ])
                    vx = msg.twist.twist.linear.x
                    vy = msg.twist.twist.linear.y
                    speed = np.sqrt(vx**2 + vy**2)
                    odom_vec = np.concatenate([pos, ori, np.array([speed])]).astype(np.float32)
                    odom_data.append(odom_vec)
                    odom_times.append(timestamp)

    except Exception as e:
        print(f"[PID:{pid} ERROR] {bag_name}: Failed to read bag file. {e}")
        return

    if not (image_data and cmd_data and odom_data):
        print(f"[PID:{pid} WARN] {bag_name}: insufficient data, skipped.")
        return

    image_times = np.array(image_times)
    cmd_data, cmd_times = np.array(cmd_data), np.array(cmd_times)
    odom_data, odom_times = np.array(odom_data), np.array(odom_times)

    synced = []
    for i, itime in enumerate(image_times):
        idx_cmd = np.argmin(np.abs(cmd_times - itime))
        idx_odom = np.argmin(np.abs(odom_times - itime))
        synced.append((i, idx_cmd, idx_odom))

    samples = []
    start_idx = history_len * history_step
    end_idx = len(synced) - future_len * future_step
    if start_idx >= end_idx:
        print(f"[PID:{pid} WARN] {bag_name}: Not enough synced data to create samples.")
        return

    for i in range(start_idx, end_idx):
        past_odoms_raw = [odom_data[synced[i - h * history_step][2]] for h in range(history_len)]
        past_odoms = np.stack(past_odoms_raw[::-1])

        future_cmds_raw, future_odoms_global_raw = [], []
        for f in range(1, future_len + 1):
            _, idx_cmd, idx_odom = synced[i + f * future_step]
            future_cmds_raw.append(cmd_data[idx_cmd])
            future_odoms_global_raw.append(odom_data[idx_odom])
        future_cmds = np.stack(future_cmds_raw)
        future_odoms_global = np.stack(future_odoms_global_raw)

        img_idx, _, current_odom_idx = synced[i]
        img = image_data[img_idx]
        current_odom_8d = odom_data[current_odom_idx]

        current_pose_7d = current_odom_8d[:7]
        future_poses_7d = future_odoms_global[:, :7]
        future_speed = future_odoms_global[:, 7]
        future_path_local_xy = transform_global_to_local(current_pose_7d, future_poses_7d)
        future_trajectory = np.hstack((future_path_local_xy, future_speed.reshape(-1, 1))).astype(np.float32)

        samples.append((img, past_odoms, future_cmds, future_trajectory))

    img_dir = out_dir / "images"
    img_dir.mkdir(exist_ok=True)
    if not samples:
        print(f"[PID:{pid} WARN] {bag_name}: No samples were created (check lengths and steps).")
        return

    for i, (img, past_odoms, future_cmds, future_trajectory) in enumerate(samples):
        cv2.imwrite(str(img_dir / f"{i:06d}.png"), img)
        np.save(out_dir / f"past_odoms_{i:06d}.npy", past_odoms)
        np.save(out_dir / f"future_cmds_{i:06d}.npy", future_cmds)
        np.save(out_dir / f"future_trajectory_{i:06d}.npy", future_trajectory)

    print(f"[PID:{pid} SAVE] {bag_name}: {len(samples)} samples saved "
          f"(H{history_len}xS{history_step}, F{future_len}xS{future_step})")


def main():
    parser = argparse.ArgumentParser(description="Extract temporal image-odom-cmd samples from ROS2 bag")
    parser.add_argument("--bags_dir", required=True, help="Path to directory containing rosbag folders (recursive search supported)")
    parser.add_argument("--outdir", required=True, help="Output directory path")
    parser.add_argument("--image_topic", default="/realsense2_camera/color/image_raw", help="Image topic name")
    parser.add_argument("--cmd_topic", default="/jetracer/cmd_drive", help="AckermannDriveStamped topic")
    parser.add_argument("--odom_topic", default="/visual_slam/tracking/odometry", help="Odometry topic")

    parser.add_argument("--history_len", type=int, default=10)
    parser.add_argument("--history_step", type=int, default=1)
    parser.add_argument("--future_len", type=int, default=30)
    parser.add_argument("--future_step", type=int, default=3)
    parser.add_argument("--workers", type=int, default=None)
    args = parser.parse_args()

    bags_dir = Path(args.bags_dir).expanduser().resolve()

    # --- 再帰的にrosbagフォルダを探索 ---
    bag_dirs = []
    for p in bags_dir.rglob("metadata.yaml"):
        if p.is_file():
            bag_dirs.append(p.parent)

    if not bag_dirs:
        print(f"[ERROR] No valid rosbag directories found in {bags_dir}.")
        if (bags_dir / "metadata.yaml").exists():
            print(f"[INFO] Treating {bags_dir} as a single bag directory.")
            bag_dirs = [bags_dir]
        else:
            return

    print(f"[INFO] Found {len(bag_dirs)} rosbag directories (recursive search).")

    tasks = []
    for bag_path in sorted(bag_dirs):
        print(f"--- Queuing {bag_path.name} ---")
        tasks.append((
            bag_path, args.outdir, args.image_topic, args.cmd_topic, args.odom_topic,
            args.history_len, args.history_step, args.future_len, args.future_step
        ))

    if args.workers:
        num_workers = args.workers
    else:
        cpu_count = os.cpu_count()
        num_workers = min(max(1, (cpu_count or 4) - 1), 8)

    print(f"[INFO] Starting parallel processing with {num_workers} workers...")
    try:
        with multiprocessing.Pool(processes=num_workers) as pool:
            pool.starmap(extract_temporal_samples, tasks)
        print("[INFO] All processing finished.")
    except Exception as e:
        print(f"[ERROR] An error occurred during parallel processing: {e}")


if __name__ == "__main__":
    try:
        multiprocessing.set_start_method("spawn", force=True)
    except RuntimeError as e:
        if "context has already been set" not in str(e):
            print(f"[WARN] Could not set start method 'spawn': {e}")
    main()
