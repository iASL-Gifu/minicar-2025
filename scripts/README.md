# グローバルローカライゼーション用マップ作成手順

ロボットの自己位置推定（グローバルローカライゼーション）のためのマップを作成する手順をまとめます。

---

## 作業の全体フロー
```
1. カメラデータ取得 (Tamiya実機) 
   ↓
2. Poseデータ作成 (ローカルPC)
   ↓
3. 特徴量抽出
   ↓
4. マップ作成
   ↓
5. セクション分割
   ↓
6. マップやyamlを配置する
```

---

## 1. カメラデータを取得

### 手順
```bash
# Dockerコンテナを起動
./run_docker.sh

# Dockerコンテナ内でtmuxセッション開始
/scripts/tmux.sh

# tmux内でシステム起動
ros2 launch system_launch base_system.launch.xml record:=false
ros2 launch bag_manager_py bag_manager_node.launch.xml
/scripts/global_localization/1.camera.sh
```

### 記録されるトピック

設定ファイル: [`bag_manager_py/config/global_localization/bag_manager.param.yaml`](https://github.com/iASL-Gifu/minicar-2025/blob/localization/ros2_ws/src/original_pkg/core/bag_manager_py/config/global_localization/bag_manager.param.yaml)

- `/infra1/image_rect_raw_mono` - 赤外線左カメラの補正済み画像
- `/left/camera_info_rect` - 左カメラの校正情報
- `/infra2/image_rect_raw_mono` - 赤外線右カメラの補正済み画像
- `/right/camera_info_rect_fixed` - 右カメラの校正情報
- `/camera/imu` - IMUデータ
- `/tf_static` - 静的座標変換
- `/tf` - 動的座標変換

---

## 2. Poseデータを作成

記録したカメラデータから、ロボットの位置・姿勢（Pose）データを生成します。

### 手順
```bash
# Dockerコンテナを起動
./run_docker.sh

# tmuxセッション開始
/scripts/global_localization/2.tmux.sh

# 各tmuxペインで以下を順番に実行
# 1. データ記録の準備
/scripts/global_localization/2.record_data.sh

# 2. Visual SLAMノード起動
/scripts/global_localization/2.vslam.sh

# 3. 元データの再生（Pose計算）
/scripts/global_localization/2.play_data.sh

# データ再生完了後
# rosbagを分割
/scripts/global_localization/2.split_data.sh
```

### 出力されるrosbag

#### `camera_only.bag`

- `/infra1/image_rect_raw_mono` - 赤外線左カメラの補正済み画像
- `/left/camera_info_rect` - 左カメラの校正情報
- `/infra2/image_rect_raw_mono` - 赤外線右カメラの補正済み画像
- `/right/camera_info_rect_fixed` - 右カメラの校正情報
- `/tf_static` - 静的座標変換
- `/tf` - 動的座標変換

#### `pose_only.bag`

- `/visual_slam/tracking/odometry` - ロボットのオドメトリ（位置・姿勢・速度情報）

---

## 3. 特徴量を抽出

画像から特徴点（キーポイント）を抽出し、マップ作成の基礎データを準備します。

### 手順
```bash
# Dockerコンテナを起動
./run_docker.sh

# 特徴量抽出スクリプト実行
/scripts/global_localization/3.extract_feature.sh
```

## 4. マップを作成

グローバルローカライゼーション用のマップを構築します。

### 手順
```bash
# Dockerコンテナを起動
./run_docker.sh

# マップ作成スクリプト実行
/scripts/global_localization/4.create_map.sh
```

---

## 5. セクションを分ける

作成したマップを複数のセクション（区画）に分割し、効率的な位置推定を可能にします。

### 手順

#### ステップ1: オドメトリデータの記録
```bash
# Dockerコンテナを起動
./run_docker.sh

# セクション検出スクリプト起動（記録モード）
/scripts/global_localization/5.sector_detector.sh
# → Select mode: "Record odometry data" を選択

# 別ターミナルでrosbagを再生
/scripts/global_localization/2.play_data.sh
# ※ ステップ2で使用したものと同じrosbagを再生
```

#### ステップ2: オドメトリの可視化とセクション定義
```bash
# 再生完了後、再度セクション検出スクリプト起動（プロットモード）
/scripts/global_localization/5.sector_detector.sh
# → Select mode: "Plot odometry data" を選択
```

#### ステップ3: セクション座標の設定

プロットを確認しながら、各セクションの中心座標(x, y)と半径を設定ファイルに記述します。

**設定ファイル**: [`localization_launch/config/section_detector.yaml`](https://github.com/iASL-Gifu/minicar-2025/blob/localization/ros2_ws/src/launch/localization_launch/config/section_detector.yaml)
```yaml
/**:
  ros__parameters:
    circles: [
      21.3, 2.3, 3.0,      # section 0
      20.654, 28.091, 3.0, # section 1
      18.4, 55.3, 3.0,     # section 2
      -6.8, 55.8, 3.0,     # section 3
      -32.3, 52.2, 3.0,    # section 4
      -32.5, 27.9, 3.0,    # section 5
      -30.7, 10.2, 3.0,    # section 6
      -28.5, -1.2, 3.0,    # section 7
      -3.1, -0.9, 3.0      # section 8
    ]
    publish_rate_hz: 2.0   # セクション情報の配信レート（2Hz = 0.5秒ごと）
```
---
