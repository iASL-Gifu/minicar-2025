import argparse
from pathlib import Path
import numpy as np
from rosbags.highlevel import AnyReader
import cv2
from scipy.spatial.transform import Rotation as R


def transform_path_to_local(current_pose_7d: np.ndarray, future_path_7d: np.ndarray) -> np.ndarray:
    """
    未来のグローバルパスを現在のロボットのローカル座標系（base_link）に変換します。

    Args:
        current_pose_7d (np.ndarray): 現在の姿勢 [x, y, z, qx, qy, qz, qw]
        future_path_7d (np.ndarray): 未来のパス (N, 7) [x, y, z, qx, qy, qz, qw]

    Returns:
        np.ndarray: ローカル座標系に変換されたパス (N, 2)。x, y座標のみを返します。
    """
    current_position = current_pose_7d[:3]
    current_quat = current_pose_7d[3:]

    future_positions = future_path_7d[:, :3]

    translated_positions = future_positions - current_position
    inv_rotation = R.from_quat(current_quat).inv()
    local_path_3d = inv_rotation.apply(translated_positions)

    return local_path_3d[:, :2].astype(np.float32)


def extract_and_save_per_bag(
    bag_path: Path,
    output_dir: Path,
    image_topic: str,
    odom_topic: str,
    future_steps: int,
    future_interval: float,
    past_steps: int,
    past_interval: float,
):
    """
    単一のrosbagから画像、過去のOdometry履歴、未来のローカルパスを抽出し、保存する。
    """
    print(f"\n[INFO] Processing bag: {bag_path.name}")
    bag_name = bag_path.name
    out_dir = output_dir / bag_name
    out_dir.mkdir(parents=True, exist_ok=True)

    image_data, image_times = [], []
    # odometryはpose(7d) + vx(1d) = 8次元で保存
    odom_data, odom_times = [], []

    try:
        with AnyReader([bag_path]) as reader:
            connections = [c for c in reader.connections if c.topic in [image_topic, odom_topic]]

            for conn, timestamp, raw in reader.messages(connections=connections):
                msg = reader.deserialize(raw, conn.msgtype)

                if conn.topic == image_topic and conn.msgtype == 'sensor_msgs/msg/Image':
                    encoding = msg.encoding
                    if encoding == 'mono8':
                        shape = (msg.height, msg.width)
                    elif encoding in ['bgr8', 'rgb8']:
                        shape = (msg.height, msg.width, 3)
                    else:
                        continue

                    image_np = np.frombuffer(msg.data, dtype=np.uint8).reshape(shape)
                    if encoding == 'rgb8':
                        image_np = cv2.cvtColor(image_np, cv2.COLOR_RGB2BGR)

                    image_data.append(image_np)
                    image_times.append(timestamp)

                elif conn.topic == odom_topic and conn.msgtype == 'nav_msgs/msg/Odometry':
                    pose = msg.pose.pose
                    position = np.array([pose.position.x, pose.position.y, pose.position.z])
                    orientation = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
                    velocity_x = msg.twist.twist.linear.x

                    odom_vec = np.concatenate([position, orientation, [velocity_x]]).astype(np.float32)
                    odom_data.append(odom_vec)
                    odom_times.append(timestamp)
    except Exception as e:
        print(f"[ERROR] Failed to read bag {bag_path.name}: {e}")
        return

    if not image_data or not odom_data:
        print(f'[WARN] Skipping {bag_name}: insufficient image or odometry data.')
        return

    image_times = np.array(image_times)
    odom_data = np.array(odom_data)
    odom_times = np.array(odom_times)

    synced_images = []
    synced_past_odoms = []
    synced_local_paths_with_vel = []

    for i, itime in enumerate(image_times):
        # --- 1. 現在のオドメトリを取得 ---
        current_odom_idx = np.argmin(np.abs(odom_times - itime))
        current_pose = odom_data[current_odom_idx, :7]

        # --- 2. 未来のパスを取得 ---
        future_path_indices = []
        is_future_path_valid = True
        for k in range(1, future_steps + 1):
            future_time = itime + (k * future_interval * 1e9)
            search_indices = np.where(odom_times >= itime)[0]
            if len(search_indices) == 0:
                is_future_path_valid = False
                break
            relative_idx = np.argmin(np.abs(odom_times[search_indices] - future_time))
            absolute_idx = search_indices[relative_idx]
            future_path_indices.append(absolute_idx)

        # --- 3. 過去のオドメトリ履歴を取得 ---
        past_odom_indices = []
        is_past_valid = True
        for k in range(past_steps):
            past_time = itime - (k * past_interval * 1e9)
            search_indices = np.where(odom_times <= itime)[0]
            if len(search_indices) == 0:
                is_past_valid = False
                break
            relative_idx = np.argmin(np.abs(odom_times[search_indices] - past_time))
            absolute_idx = search_indices[relative_idx]
            past_odom_indices.append(absolute_idx)

        # 不完全なデータはスキップ
        if not is_future_path_valid or not is_past_valid or \
           len(future_path_indices) != future_steps or \
           len(past_odom_indices) != past_steps:
            continue

        # --- 4. データを整形 ---
        global_future_poses = odom_data[future_path_indices, :7]
        future_velocities = odom_data[future_path_indices, 7:]
        local_path_xy = transform_path_to_local(current_pose, global_future_poses)
        local_path_with_vel = np.concatenate([local_path_xy, future_velocities], axis=1)

        # 過去オドメトリを時系列順（古い→新しい）に並べ替え
        past_odom_indices.reverse()
        past_odoms = odom_data[past_odom_indices]

        synced_images.append(image_data[i])
        synced_past_odoms.append(past_odoms)
        synced_local_paths_with_vel.append(local_path_with_vel)

    if not synced_images:
        print(f'[WARN] Skipping {bag_name}: No valid synchronized data pairs found.')
        return

    # --- 5. 保存処理 ---
    images_save_dir = out_dir / 'images'
    images_save_dir.mkdir(exist_ok=True)
    for i, image in enumerate(synced_images):
        cv2.imwrite(str(images_save_dir / f"{i:06d}.png"), image)

    np.save(out_dir / 'paths.npy', np.array(synced_local_paths_with_vel, dtype=np.float32))
    np.save(out_dir / 'past_odoms.npy', np.array(synced_past_odoms, dtype=np.float32))

    print(f'[SAVE] {bag_name}: Saved {len(synced_images)} samples to {out_dir}')


def main():
    parser = argparse.ArgumentParser(
        description='Extract synchronized image, past odometry, and future local path data from ROS 2 bags.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )

    # --- 入出力関連 ---
    parser.add_argument('--bags-dir', required=True, type=Path,
                        help='Path to the directory containing ROS 2 bag directories.')
    parser.add_argument('--outdir', required=True, type=Path,
                        help='Root directory to save the processed dataset.')

    # --- トピック名 ---
    parser.add_argument('--image-topic', default='/realsense2_camera/color/image_raw',
                        help='Image topic name.')
    parser.add_argument('--odom-topic', default='/visual_slam/tracking/odometry',
                        help='Odometry topic name.')

    # --- データセット生成パラメータ ---
    parser.add_argument('--past-steps', type=int, default=5,
                        help='Number of past odometry points to retrieve (before current timestamp).')
    parser.add_argument('--past-interval', type=float, default=0.1,
                        help='Time interval [s] between consecutive past odometry points.')
    parser.add_argument('--future-steps', type=int, default=10,
                        help='Number of future trajectory points to generate (after current timestamp).')
    parser.add_argument('--future-interval', type=float, default=0.333,
                        help='Time interval [s] between consecutive future trajectory points.')

    args = parser.parse_args()

    bags_dir = args.bags_dir.expanduser().resolve()
    bag_paths = sorted([p for p in bags_dir.iterdir() if p.is_dir() and (p / 'metadata.yaml').exists()])

    if not bag_paths:
        print(f"[ERROR] No rosbag directories (containing metadata.yaml) found in {bags_dir}")
        return

    print(f"[INFO] Found {len(bag_paths)} rosbag directories.")

    for bag_path in bag_paths:
        extract_and_save_per_bag(
            bag_path=bag_path,
            output_dir=args.outdir,
            image_topic=args.image_topic,
            odom_topic=args.odom_topic,
            future_steps=args.future_steps,
            future_interval=args.future_interval,
            past_steps=args.past_steps,
            past_interval=args.past_interval
        )

    print("\n[INFO] All processing finished.")


if __name__ == '__main__':
    main()
