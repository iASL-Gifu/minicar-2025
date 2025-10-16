#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import sys
from pathlib import Path

def plot_jetson_monitor(csv_file):
    """
    Jetson Monitor CSV ファイルをグラフで表示
    
    Args:
        csv_file (str): CSV ファイルのパス
    """
    
    # ファイルの存在確認
    if not Path(csv_file).exists():
        print(f"Error: File not found - {csv_file}")
        sys.exit(1)
    
    # CSV を読み込み
    try:
        df = pd.read_csv(csv_file)
    except Exception as e:
        print(f"Error reading CSV: {e}")
        sys.exit(1)
    
    # タイムスタンプを datetime に変換
    df['timestamp'] = pd.to_datetime(df['timestamp'])
    
    # 経過時間（秒）を計算
    elapsed_time = (df['timestamp'] - df['timestamp'].iloc[0]).dt.total_seconds()
    
    # グラフの設定
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    fig.suptitle('Jetson System Monitor', fontsize=16, fontweight='bold')
    
    # CPU グラフ
    axes[0].plot(elapsed_time, df['cpu_percent'], marker='o', linewidth=2, color='#FF6B6B', label='CPU')
    axes[0].fill_between(elapsed_time, df['cpu_percent'], alpha=0.3, color='#FF6B6B')
    axes[0].set_ylabel('CPU Usage (%)', fontsize=11, fontweight='bold')
    axes[0].set_ylim(0, 100)
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc='upper right')
    
    # 統計情報を表示
    cpu_avg = df['cpu_percent'].mean()
    cpu_max = df['cpu_percent'].max()
    cpu_min = df['cpu_percent'].min()
    axes[0].text(0.02, 0.95, f'Avg: {cpu_avg:.1f}% | Max: {cpu_max:.1f}% | Min: {cpu_min:.1f}%',
                 transform=axes[0].transAxes, fontsize=10, verticalalignment='top',
                 bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
    
    # RAM グラフ
    axes[1].plot(elapsed_time, df['ram_percent'], marker='s', linewidth=2, color='#4ECDC4', label='RAM')
    axes[1].fill_between(elapsed_time, df['ram_percent'], alpha=0.3, color='#4ECDC4')
    axes[1].set_ylabel('RAM Usage (%)', fontsize=11, fontweight='bold')
    axes[1].set_ylim(0, 100)
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc='upper right')
    
    # 統計情報を表示
    ram_avg = df['ram_percent'].mean()
    ram_max = df['ram_percent'].max()
    ram_min = df['ram_percent'].min()
    axes[1].text(0.02, 0.95, f'Avg: {ram_avg:.1f}% | Max: {ram_max:.1f}% | Min: {ram_min:.1f}%',
                 transform=axes[1].transAxes, fontsize=10, verticalalignment='top',
                 bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.5))
    
    # SWAP グラフ
    axes[2].plot(elapsed_time, df['swap_percent'], marker='^', linewidth=2, color='#95E1D3', label='SWAP')
    axes[2].fill_between(elapsed_time, df['swap_percent'], alpha=0.3, color='#95E1D3')
    axes[2].set_xlabel('Elapsed Time (seconds)', fontsize=11, fontweight='bold')
    axes[2].set_ylabel('SWAP Usage (%)', fontsize=11, fontweight='bold')
    axes[2].set_ylim(0, 100)
    axes[2].grid(True, alpha=0.3)
    axes[2].legend(loc='upper right')
    
    # 統計情報を表示
    swap_avg = df['swap_percent'].mean()
    swap_max = df['swap_percent'].max()
    swap_min = df['swap_percent'].min()
    axes[2].text(0.02, 0.95, f'Avg: {swap_avg:.1f}% | Max: {swap_max:.1f}% | Min: {swap_min:.1f}%',
                 transform=axes[2].transAxes, fontsize=10, verticalalignment='top',
                 bbox=dict(boxstyle='round', facecolor='lightgreen', alpha=0.5))
    
    plt.tight_layout()
    
    # グラフを JPG で保存（CSVファイルと同じディレクトリに保存）
    csv_path = Path(csv_file)
    output_file = csv_path.parent / (csv_path.stem + '_graph.jpg')
    plt.savefig(output_file, format='jpg', dpi=150, bbox_inches='tight')
    print(f"\nGraph saved: {output_file}")
    
    # グラフを表示
    plt.show()
    
    # 統計情報を出力
    print("\n=== Statistics ===")
    print(f"Data Points: {len(df)}")
    print(f"Duration: {elapsed_time.iloc[-1]:.1f} seconds")
    print(f"\nCPU  - Avg: {cpu_avg:.1f}% | Min: {cpu_min:.1f}% | Max: {cpu_max:.1f}%")
    print(f"RAM  - Avg: {ram_avg:.1f}% | Min: {ram_min:.1f}% | Max: {ram_max:.1f}%")
    print(f"SWAP - Avg: {swap_avg:.1f}% | Min: {swap_min:.1f}% | Max: {swap_max:.1f}%")
    
    # メモリ使用量の詳細
    print(f"\nRAM Details:")
    print(f"  Used (Avg): {df['ram_used_mb'].mean():.0f} MB")
    print(f"  Total: {df['ram_total_mb'].iloc[0]} MB")
    print(f"\nSWAP Details:")
    print(f"  Used (Avg): {df['swap_used_mb'].mean():.0f} MB")
    print(f"  Total: {df['swap_total_mb'].iloc[0]} MB")

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 plot_monitor.py <csv_file>")
        print("\nExample:")
        print("  python3 plot_monitor.py jetson_monitor_20251016_090000.csv")
        sys.exit(1)
    
    csv_file = sys.argv[1]
    plot_jetson_monitor(csv_file)