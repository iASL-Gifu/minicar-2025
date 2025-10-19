#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.widgets import Cursor
import sys

class PathPlotter:
    def __init__(self, csv_filename, downsample_factor=10):
        # CSVファイルを読み込み
        self.df_original = pd.read_csv(csv_filename)
        
        # データを間引く（downsample_factor点に1点を表示）
        self.df = self.df_original.iloc[::downsample_factor].reset_index(drop=True)
        
        print(f"Loaded {len(self.df_original)} data points from {csv_filename}")
        print(f"Displaying {len(self.df)} points (downsampled by factor of {downsample_factor})")
        
        # プロット用の図を作成
        self.fig, self.ax = plt.subplots(figsize=(12, 10))
        
        # 経路をプロット
        self.scatter = self.ax.scatter(self.df['x'], self.df['y'], 
                                       c=range(len(self.df)), 
                                       cmap='viridis', 
                                       s=10, 
                                       alpha=0.6)
        
        # カラーバー（時系列を表示）
        cbar = plt.colorbar(self.scatter, ax=self.ax)
        cbar.set_label('Time progression', rotation=270, labelpad=20)
        
        # 軸ラベルとタイトル
        self.ax.set_xlabel('X [m]', fontsize=12)
        self.ax.set_ylabel('Y [m]', fontsize=12)
        self.ax.set_title('Robot Path from Odometry', fontsize=14)
        self.ax.grid(True, alpha=0.3)
        self.ax.set_aspect('equal', adjustable='datalim')
        
        # 開始点と終了点をマーク
        self.ax.plot(self.df['x'].iloc[0], self.df['y'].iloc[0], 
                    'go', markersize=12, label='Start', zorder=5)
        self.ax.plot(self.df['x'].iloc[-1], self.df['y'].iloc[-1], 
                    'ro', markersize=12, label='End', zorder=5)
        self.ax.legend()
        
        # カーソルを追加（クロスヘア）
        self.cursor = Cursor(self.ax, useblit=True, color='red', linewidth=1)
        
        # マウスクリックイベントを設定
        self.fig.canvas.mpl_connect('button_press_event', self.on_click)
        
        # 座標表示用のテキスト
        self.coord_text = self.ax.text(0.02, 0.98, '', 
                                       transform=self.ax.transAxes,
                                       verticalalignment='top',
                                       bbox=dict(boxstyle='round', 
                                               facecolor='wheat', 
                                               alpha=0.8),
                                       fontsize=10)
        
        print("\nClick on any point to see its coordinates!")
        print("Close the window to exit.")
        
    def on_click(self, event):
        """マウスクリック時に最も近い点の座標を表示"""
        if event.inaxes != self.ax:
            return
        
        # クリックした位置
        click_x = event.xdata
        click_y = event.ydata
        
        if click_x is None or click_y is None:
            return
        
        # 元のデータから最も近い点を探す（より正確な座標取得のため）
        distances = (self.df_original['x'] - click_x)**2 + (self.df_original['y'] - click_y)**2
        nearest_idx = distances.idxmin()
        
        # 最も近い点の情報を取得
        nearest_x = self.df_original.loc[nearest_idx, 'x']
        nearest_y = self.df_original.loc[nearest_idx, 'y']
        nearest_z = self.df_original.loc[nearest_idx, 'z']
        nearest_time = self.df_original.loc[nearest_idx, 'timestamp']
        
        # 座標を表示
        coord_info = (f"Point #{nearest_idx}\n"
                     f"X: {nearest_x:.3f} m\n"
                     f"Y: {nearest_y:.3f} m\n"
                     f"Z: {nearest_z:.3f} m\n"
                     f"Time: {nearest_time:.2f} s")
        
        self.coord_text.set_text(coord_info)
        
        # コンソールにも出力
        print(f"\n--- Clicked Point Info ---")
        print(f"Index: {nearest_idx}")
        print(f"X: {nearest_x:.3f} m")
        print(f"Y: {nearest_y:.3f} m")
        print(f"Z: {nearest_z:.3f} m")
        print(f"Timestamp: {nearest_time:.2f} s")
        
        # 図を更新
        self.fig.canvas.draw()
    
    def show(self):
        """プロットを表示"""
        plt.tight_layout()
        plt.show()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 plot_odometry_path.py <csv_filename> [downsample_factor]")
        print("Example: python3 plot_odometry_path.py odometry_path_20250101_120000.csv 10")
        print("         downsample_factor: Display every Nth point (default: 10)")
        sys.exit(1)
    
    csv_file = sys.argv[1]
    
    # 間引き率を取得（デフォルトは10）
    downsample_factor = 10
    if len(sys.argv) >= 3:
        try:
            downsample_factor = int(sys.argv[2])
            if downsample_factor < 1:
                print("Error: downsample_factor must be >= 1")
                sys.exit(1)
        except ValueError:
            print("Error: downsample_factor must be an integer")
            sys.exit(1)
    
    try:
        plotter = PathPlotter(csv_file, downsample_factor)
        plotter.show()
    except FileNotFoundError:
        print(f"Error: File '{csv_file}' not found!")
        sys.exit(1)
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)