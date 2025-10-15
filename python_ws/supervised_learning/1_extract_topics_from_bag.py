from pathlib import Path
import numpy as np
from rosbags.highlevel import AnyReader
import cv2

def extract_and_save_per_bag(bag_path, output_dir, image_topic, cmd_topic, odom_topic):
    """
    単一のrosbagファイルから画像、制御指令、オドメトリデータを抽出し、
    タイムスタンプを同期させて保存する関数。
    """
    bag_path = Path(bag_path).expanduser().resolve()
    bag_name = bag_path.name
    out_dir = Path(output_dir) / bag_name
    out_dir.mkdir(parents=True, exist_ok=True)

    image_data, image_times = [], []
    cmd_data, cmd_times = [], []
    odom_data, odom_times = [], []  # オドメトリデータを格納するリストを追加

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
            
            # === Odometryメッセージを処理するブロックを追加 ===
            elif conn.topic == odom_topic and conn.msgtype == 'nav_msgs/msg/Odometry':
                pose = msg.pose.pose
                # 位置(x, y, z)と姿勢(quaternion: x, y, z, w)を抽出
                position = np.array([pose.position.x, pose.position.y, pose.position.z])
                orientation = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
                
                # 位置と姿勢を結合して1つのNumpy配列にする
                odom_vec = np.concatenate([position, orientation]).astype(np.float32)
                odom_data.append(odom_vec)
                odom_times.append(timestamp)
    
    # 3種類のデータが1つでも欠けていたらスキップ
    if len(image_data) == 0 or len(cmd_data) == 0 or len(odom_data) == 0:
        print(f'[WARN] Skipping {bag_name}: insufficient data (images, commands, or odometry)')
        return

    image_times = np.array(image_times)
    cmd_data, cmd_times = np.array(cmd_data), np.array(cmd_times)
    odom_data, odom_times = np.array(odom_data), np.array(odom_times) # オドメトリデータをNumpy配列に変換

    synced_images, synced_steers, synced_speeds, synced_odoms = [], [], [], [] # 同期後のオドメトリ用リストを追加

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
    
    print(f'[SAVE] {bag_name}: {len(synced_images)} samples saved to {out_dir}')


def extract_all_bags_in_dir(bags_dir, output_dir, image_topic, cmd_topic, odom_topic):
    """
    指定されたディレクトリ内の全てのrosbagを展開する関数。
    """
    bags_dir = Path(bags_dir).expanduser().resolve()
    bag_dirs = sorted([p for p in bags_dir.iterdir() if (p / 'metadata.yaml').exists()])

    print(f"[INFO] Found {len(bag_dirs)} rosbag directories.")
    for bag_path in bag_dirs:
        extract_and_save_per_bag(bag_path, output_dir, image_topic, cmd_topic, odom_topic)


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Extract and synchronize image, command, and odometry data from rosbags.')
    parser.add_argument('--bags_dir', required=True, help='Path to directory containing rosbag folders')
    parser.add_argument('--outdir', required=True, help='Output root directory')
    parser.add_argument('--image_topic', default='/realsense2_camera/color/image_raw', help='Image topic name')
    parser.add_argument('--cmd_topic', default='/jetracer/cmd_drive', help='Command topic name')
    parser.add_argument('--odom_topic', default='/visual_slam/tracking/odometry', help='Odometry topic name')
    args = parser.parse_args()

    extract_all_bags_in_dir(args.bags_dir, args.outdir, args.image_topic, args.cmd_topic, args.odom_topic)