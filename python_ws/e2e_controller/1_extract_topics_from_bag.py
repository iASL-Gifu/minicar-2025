import argparse
import multiprocessing  # 並列処理のため追加
import os               # プロセスID取得のため追加
from pathlib import Path

import cv2
import numpy as np
from rosbags.highlevel import AnyReader


def extract_and_save_per_bag(bag_path, output_dir, image_topic, cmd_topic, odom_topic):
    """
    (変更) 単一のrosbagファイルからデータを抽出する並列ワーカー関数。
    """
    pid = os.getpid()  # プロセスIDを取得
    bag_path = Path(bag_path).expanduser().resolve()
    bag_name = bag_path.name
    out_dir = Path(output_dir) / bag_name
    out_dir.mkdir(parents=True, exist_ok=True)

    image_data, image_times = [], []
    cmd_data, cmd_times = [], []
    odom_data, odom_times = [], []

    try:
        with AnyReader([bag_path]) as reader:
            # 読み込むトピックにオドメトリのトピックを追加
            connections = [c for c in reader.connections if c.topic in [image_topic, cmd_topic, odom_topic]]

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

                elif conn.topic == cmd_topic and conn.msgtype == 'ackermann_msgs/msg/AckermannDriveStamped':
                    cmd_data.append(np.array([msg.drive.steering_angle, msg.drive.speed], dtype=np.float32))
                    cmd_times.append(timestamp)

                elif conn.topic == odom_topic and conn.msgtype == 'nav_msgs/msg/Odometry':
                    pose = msg.pose.pose
                    # 位置(x, y, z)と姿勢(quaternion: x, y, z, w)を抽出
                    position = np.array([pose.position.x, pose.position.y, pose.position.z])
                    orientation = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])

                    # 位置と姿勢を結合して1つのNumpy配列にする
                    odom_vec = np.concatenate([position, orientation]).astype(np.float32)
                    odom_data.append(odom_vec)
                    odom_times.append(timestamp)

    except Exception as e:
        print(f"[PID:{pid} ERROR] {bag_name}: Failed to read bag file. {e}")
        return

    # 3種類のデータが1つでも欠けていたらスキップ
    if len(image_data) == 0 or len(cmd_data) == 0 or len(odom_data) == 0:
        print(f'[PID:{pid} WARN] Skipping {bag_name}: insufficient data (images, commands, or odometry)')
        return

    image_times = np.array(image_times)
    cmd_data, cmd_times = np.array(cmd_data), np.array(cmd_times)
    odom_data, odom_times = np.array(odom_data), np.array(odom_times)

    synced_images, synced_steers, synced_speeds, synced_odoms = [], [], [], []

    # 画像のタイムスタンプを基準に、最も近い時刻の制御指令とオドメトリを同期
    for i, itime in enumerate(image_times):
        idx_cmd = np.argmin(np.abs(cmd_times - itime))
        idx_odom = np.argmin(np.abs(odom_times - itime))

        synced_images.append(image_data[i])
        synced_steers.append(cmd_data[idx_cmd][0])
        synced_speeds.append(cmd_data[idx_cmd][1])
        synced_odoms.append(odom_data[idx_odom])

    # 画像を保存
    images_save_dir = out_dir / 'images'
    images_save_dir.mkdir(exist_ok=True)

    for i, image in enumerate(synced_images):
        image_filename = f"{i:06d}.png"
        image_save_path = images_save_dir / image_filename
        cv2.imwrite(str(image_save_path), image)

    # 同期したデータをNumpy形式で保存
    np.save(out_dir / 'steers.npy', np.array(synced_steers))
    np.save(out_dir / 'speeds.npy', np.array(synced_speeds))
    np.save(out_dir / 'odoms.npy', np.array(synced_odoms))

    print(f'[PID:{pid} SAVE] {bag_name}: {len(synced_images)} samples saved to {out_dir}')


def main():
    """
    メイン関数。引数解析、バッグ検索、並列処理の起動を行う。
    """
    parser = argparse.ArgumentParser(description='Extract and synchronize image, command, and odometry data from rosbags.')
    parser.add_argument('--bags_dir', required=True, help='Path to directory containing rosbag folders')
    parser.add_argument('--outdir', required=True, help='Output root directory')
    parser.add_argument('--image_topic', default='/realsense2_camera/color/image_raw', help='Image topic name')
    parser.add_argument('--cmd_topic', default='/jetracer/cmd_drive', help='Command topic name')
    parser.add_argument('--odom_topic', default='/visual_slam/tracking/odometry', help='Odometry topic name')
    
    # 並列ワーカー数を指定するオプションを追加
    parser.add_argument('--workers', type=int, default=None, help='Number of parallel workers. (Default: CPU count - 1, max 8)')
    
    args = parser.parse_args()

    bags_dir = Path(args.bags_dir).expanduser().resolve()
    
    # バッグディレクトリの検索ロジック
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

    # --- 並列処理の準備 ---

    # 1. 処理するタスクの引数リストを作成
    tasks = []
    for bag_path in sorted(bag_dirs):
        print(f"--- Queuing {bag_path.name} ---")
        task_args = (
            bag_path,
            args.outdir,
            args.image_topic,
            args.cmd_topic,
            args.odom_topic,
        )
        tasks.append(task_args)

    # 2. ワーカー数を決定
    if args.workers:
        num_workers = args.workers
    else:
        cpu_count = os.cpu_count()
        if cpu_count:
            # CPU数-1 (最低1)、ただし最大を 8 に制限 (I/O負荷を考慮)
            num_workers = min(max(1, cpu_count - 1), 8)
        else:
            num_workers = 4  # デフォルト

    print(f"[INFO] Starting parallel processing with {num_workers} workers...")

    # 3. プロセスプールを作成してタスクを実行
    try:
        with multiprocessing.Pool(processes=num_workers) as pool:
            # starmap を使い、タスクリスト (引数のタプル) を
            # extract_and_save_per_bag 関数に展開して渡します。
            pool.starmap(extract_and_save_per_bag, tasks)

        print("[INFO] All processing finished.")

    except Exception as e:
        print(f"[ERROR] An error occurred during parallel processing: {e}")


if __name__ == '__main__':
    # 'spawn' メソッドの強制指定
    try:
        multiprocessing.set_start_method('spawn', force=True)
    except RuntimeError as e:
        if "context has already been set" not in str(e):
             print(f"[WARN] Could not set start method 'spawn': {e}")
        pass

    main()