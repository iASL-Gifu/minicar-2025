import argparse
from pathlib import Path
import numpy as np
from tqdm import tqdm
import matplotlib
# GUIバックエンドのない環境（サーバーなど）でも動作するように 'Agg' を指定します
matplotlib.use('Agg') 
import matplotlib.pyplot as plt

def analyze_dataset_statistics(dataset_dir: Path, output_dir: Path):
    """
    データセットを再帰的にスキャンし、制御コマンド(ステア、速度)の統計を計算・保存する。
    """
    dataset_dir = dataset_dir.resolve()
    output_dir = output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"[INFO] Recursively analyzing dataset statistics from: {dataset_dir}")
    print(f"[INFO] Saving plots to: {output_dir}")

    # --- 変更点 ---
    # glob.glob から pathlib.rglob に変更し、再帰的に検索
    print(f"[INFO] Searching for 'future_cmds_*.npy' files recursively...")
    cmd_files = sorted(list(dataset_dir.rglob("future_cmds_*.npy")))
    # --- 変更ここまで ---
    
    num_samples = len(cmd_files)
    
    if num_samples == 0:
        print("[ERROR] No 'future_cmds_*.npy' files found recursively. Exiting.")
        print(f"       (Searched in: {dataset_dir})")
        return
    
    print(f"[INFO] Found {num_samples} 'future_cmds_*.npy' files.")

    # 全データを蓄積するリスト
    all_steer_values = []
    all_speed_values = []
    
    # サンプルごとの統計を蓄積するリスト
    sample_avg_abs_steer = [] # 「どのくらいカーブしているか」の指標
    sample_avg_speed = []

    for f_path in tqdm(cmd_files, desc="Scanning samples"):
        try:
            # future_cmds は (F, 2) の形状 (F=future_len, 2=[steer, speed])
            future_cmds = np.load(f_path) 
            
            # 1. 全ステップの値を蓄積 (ヒストグラム用)
            all_steer_values.extend(future_cmds[:, 0])
            all_speed_values.extend(future_cmds[:, 1])
            
            # 2. サンプルごとの統計を計算 (カーブの閾値判断用)
            # サンプルの「平均絶対ステア」を計算
            sample_avg_abs_steer.append(np.mean(np.abs(future_cmds[:, 0])))
            sample_avg_speed.append(np.mean(future_cmds[:, 1]))
            
        except Exception as e:
            print(f"[WARN] Failed to load or process {f_path}: {e}")
            continue

    if not all_steer_values:
        print("[ERROR] No valid data could be loaded.")
        return

    # numpy配列に変換
    all_steer = np.array(all_steer_values)
    all_speed = np.array(all_speed_values)
    sample_avg_abs_steer = np.array(sample_avg_abs_steer)
    sample_avg_speed = np.array(sample_avg_speed)

    # --- 1. 全ステップの統計を計算・表示 ---
    print("\n" + "="*30)
    print(" Overall Statistics (All Future Steps)")
    print("="*30)
    
    print(f"--- Steering (rad) ---")
    print(f"  Total steps: {len(all_steer)}")
    print(f"  Mean:   {np.mean(all_steer):.4f}")
    print(f"  Std:    {np.std(all_steer):.4f}")
    print(f"  Min:    {np.min(all_steer):.4f}")
    print(f"  Max:    {np.max(all_steer):.4f}")
    print(f"  Median: {np.median(all_steer):.4f}")
    print(f"  Abs Mean: {np.mean(np.abs(all_steer)):.4f}") # 絶対値の平均
    
    print(f"\n--- Speed (m/s) ---")
    print(f"  Total steps: {len(all_speed)}")
    print(f"  Mean:   {np.mean(all_speed):.4f}")
    print(f"  Std:    {np.std(all_speed):.4f}")
    print(f"  Min:    {np.min(all_speed):.4f}")
    print(f"  Max:    {np.max(all_speed):.4f}")
    print(f"  Median: {np.median(all_speed):.4f}")

    # --- 2. サンプルごとの統計を計算・表示 ---
    print("\n" + "="*30)
    print(f" Per-Sample Statistics (N={num_samples} samples)")
    print("="*30)
    print(f"--- Avg. Abs. Steer (rad) ---")
    print(f"  (This value indicates 'how much a sample curves')")
    print(f"  Mean (of Avg Abs Steer): {np.mean(sample_avg_abs_steer):.4f}")
    print(f"  Max (of Avg Abs Steer):  {np.max(sample_avg_abs_steer):.4f}")
    
    # 「カーブ」の閾値の提案
    p90 = np.percentile(sample_avg_abs_steer, 90)
    p75 = np.percentile(sample_avg_abs_steer, 75)
    p50 = np.percentile(sample_avg_abs_steer, 50)
    print(f"\n--- 'Curve' Threshold Suggestion (Percentiles) ---")
    print(f"  50th Percentile (Median): {p50:.4f} rad")
    print(f"  75th Percentile:          {p75:.4f} rad")
    print(f"  90th Percentile:          {p90:.4f} rad")
    print(f"  (e.g., Use > {p75:.4f} as a 'strong curve' criterion for sampling)")


    # --- 3. ヒストグラムの描画・保存 ---
    
    # (A) 全ステア値のヒストグラム (ログスケール)
    plt.figure(figsize=(12, 8))
    plt.hist(all_steer, bins=100, range=(-0.7, 0.7)) # -0.7~0.7 rad (約40度) の範囲
    plt.title(f"Distribution of All Steering Values (N={len(all_steer)})")
    plt.xlabel("Steer (rad)")
    plt.ylabel("Frequency (Log Scale)")
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.axvline(0, color='red', linestyle='--')
    plt.yscale('log') # 0が多すぎるはずなのでログスケール推奨
    plt.savefig(output_dir / "hist_all_steer_values_log_scale.png")
    plt.close()

    # (B) 全速度値のヒストグラム
    plt.figure(figsize=(12, 8))
    plt.hist(all_speed, bins=100)
    plt.title(f"Distribution of All Speed Values (N={len(all_speed)})")
    plt.xlabel("Speed (m/s)")
    plt.ylabel("Frequency")
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.savefig(output_dir / "hist_all_speed_values.png")
    plt.close()

    # (C) ★最重要★ サンプルごとの平均絶対ステア値のヒストグラム (ログスケール)
    plt.figure(figsize=(12, 8))
    plt.hist(sample_avg_abs_steer, bins=100, range=(0, np.max(sample_avg_abs_steer)))
    plt.title(f"Distribution of Per-Sample Avg. Absolute Steer (N={num_samples} samples)")
    plt.xlabel("Average Absolute Steer (rad) per Sample")
    plt.ylabel("Frequency (Number of Samples) (Log Scale)")
    plt.grid(True, linestyle='--', alpha=0.6)
    plt.axvline(p75, color='red', linestyle='--', label=f'75th Perc. ({p75:.3f})')
    plt.axvline(p90, color='orange', linestyle='--', label=f'90th Perc. ({p90:.3f})')
    plt.legend()
    plt.yscale('log') # ストレート(0付近)が多いためログスケール
    plt.savefig(output_dir / "hist_sample_avg_abs_steer_log_scale.png")
    plt.close()

    print(f"\n[INFO] Histograms saved to {output_dir}")

def main():
    parser = argparse.ArgumentParser(
        description="Analyze command statistics (steer, speed) from a trajectory dataset.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "-d", "--datadir",
        type=Path,
        required=True,
        help="Path to the base dataset directory (e.g., ./datasets) "
             "containing sequence subdirectories."
    )
    parser.add_argument(
        "-o", "--output",
        type=Path,
        default=Path("./outputs/statistics"),
        help="Directory to save the analysis plots."
    )
    args = parser.parse_args()
    analyze_dataset_statistics(args.datadir, args.output)


if __name__ == "__main__":
    main()