#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.widgets import Cursor, Button
import sys
import os
from datetime import datetime

class PathPlotter:
    def __init__(self, csv_filename, downsample_factor=10, output_csv='clicked_points.csv', default_radius=0.5):
        # CSVファイルを読み込み
        self.df_original = pd.read_csv(csv_filename)
        
        # データを間引く（downsample_factor点に1点を表示）
        self.df = self.df_original.iloc[::downsample_factor].reset_index(drop=True)
        
        # 出力用のCSVファイル名
        self.output_csv = output_csv
        
        # デフォルトの半径
        self.default_radius = default_radius
        
        # 現在のセクション番号
        self.current_section = 1
        
        # クリックした点を保存するリスト
        self.clicked_points = []
        
        # 既存の出力ファイルがあれば読み込む
        if os.path.exists(self.output_csv):
            try:
                existing_df = pd.read_csv(self.output_csv)
                
                # 既存ファイルの形式をチェック
                if 'x' in existing_df.columns and 'y' in existing_df.columns:
                    # radiusカラムがない場合はデフォルト値を追加
                    if 'radius' not in existing_df.columns:
                        print(f"Warning: 'radius' column not found in existing file. Adding default radius: {default_radius}m")
                        existing_df['radius'] = default_radius
                    
                    # sectionカラムがない場合はデフォルト値を追加
                    if 'section' not in existing_df.columns:
                        print(f"Warning: 'section' column not found in existing file. Adding default section: 1")
                        existing_df['section'] = 1
                    
                    # x, y, radius, sectionのみを抽出
                    existing_df = existing_df[['x', 'y', 'radius', 'section']]
                    self.clicked_points = existing_df.to_dict('records')
                    
                    # 最大セクション番号を取得して次のセクション番号を設定
                    if self.clicked_points:
                        self.current_section = max(p['section'] for p in self.clicked_points)
                    
                    print(f"Loaded {len(self.clicked_points)} existing points from {self.output_csv}")
                else:
                    print(f"Warning: Existing file format not compatible. Starting fresh.")
                    
            except Exception as e:
                print(f"Warning: Could not load existing file: {e}")
                print("Starting with empty points list.")
        
        print(f"Loaded {len(self.df_original)} data points from {csv_filename}")
        print(f"Displaying {len(self.df)} points (downsampled by factor of {downsample_factor})")
        print(f"Clicked points will be saved to: {self.output_csv}")
        print(f"Default radius: {self.default_radius} m")
        
        # プロット用の図を作成（ボタン用のスペースを確保）
        self.fig = plt.figure(figsize=(12, 10))
        self.ax = self.fig.add_axes([0.1, 0.2, 0.85, 0.75])
        
        # 経路をプロット
        self.scatter = self.ax.scatter(self.df['x'], self.df['y'], 
                                       c=range(len(self.df)), 
                                       cmap='viridis', 
                                       s=10, 
                                       alpha=0.6)
        
        # クリックした点を表示するためのプロット（初期は空）
        self.clicked_scatter = self.ax.scatter([], [], 
                                              c='red', 
                                              s=100, 
                                              marker='x',
                                              linewidths=3,
                                              label='Clicked Points',
                                              zorder=10)
        
        # 半径の円を表示するためのプロット
        self.radius_circles = []
        
        # カラーバー（時系列を表示）
        cbar = plt.colorbar(self.scatter, ax=self.ax)
        cbar.set_label('Time progression', rotation=270, labelpad=20)
        
        # 軸ラベルとタイトル
        self.ax.set_xlabel('X [m]', fontsize=12)
        self.ax.set_ylabel('Y [m]', fontsize=12)
        self.ax.set_title(f'Robot Path - Click to Record Points (Section: {self.current_section}, Radius: {self.default_radius}m)', fontsize=14)
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
        
        # ボタンを追加
        # 保存ボタン
        ax_save = self.fig.add_axes([0.15, 0.08, 0.12, 0.04])
        self.btn_save = Button(ax_save, 'Save Points')
        self.btn_save.on_clicked(self.save_points)
        
        # クリアボタン
        ax_clear = self.fig.add_axes([0.30, 0.08, 0.12, 0.04])
        self.btn_clear = Button(ax_clear, 'Clear All')
        self.btn_clear.on_clicked(self.clear_points)
        
        # 最後の点を削除するボタン
        ax_undo = self.fig.add_axes([0.45, 0.08, 0.12, 0.04])
        self.btn_undo = Button(ax_undo, 'Undo Last')
        self.btn_undo.on_clicked(self.undo_last)
        
        # 半径変更ボタン
        ax_radius_dec = self.fig.add_axes([0.60, 0.08, 0.05, 0.04])
        self.btn_radius_dec = Button(ax_radius_dec, 'R-')
        self.btn_radius_dec.on_clicked(self.decrease_radius)
        
        ax_radius_inc = self.fig.add_axes([0.72, 0.08, 0.05, 0.04])
        self.btn_radius_inc = Button(ax_radius_inc, 'R+')
        self.btn_radius_inc.on_clicked(self.increase_radius)
        
        # セクション変更ボタン
        ax_section = self.fig.add_axes([0.80, 0.08, 0.10, 0.04])
        self.btn_next_section = Button(ax_section, 'Next Section')
        self.btn_next_section.on_clicked(self.next_section)
        
        # ステータス表示（先に作成）
        ax_status = self.fig.add_axes([0.15, 0.03, 0.7, 0.04])
        ax_status.axis('off')
        self.status_text = ax_status.text(0.5, 0.5, 
                                         f'Section: {self.current_section} | Points: {len(self.clicked_points)} | Radius: {self.default_radius:.1f}m',
                                         ha='center', va='center',
                                         fontsize=12, fontweight='bold',
                                         bbox=dict(boxstyle='round', 
                                                 facecolor='lightblue', 
                                                 alpha=0.8))
        
        # 既存のクリック点があれば表示（status_text作成後に実行）
        if self.clicked_points:
            self.update_clicked_points_plot()
        
        print("\n=== Controls ===")
        print("Left Click: Add point")
        print("'Save Points' button: Save to CSV")
        print("'Undo Last' button: Remove last added point")
        print("'Clear All' button: Remove all points")
        print("'R-' / 'R+' buttons: Decrease/Increase radius (±0.1m)")
        print("'Next Section' button: Move to next section")
        print("Close window to exit")
        print("================\n")
    
    def increase_radius(self, event=None):
        """半径を0.1m増やす"""
        self.default_radius += 0.1
        self.update_status()
        self.ax.set_title(f'Robot Path - Click to Record Points (Section: {self.current_section}, Radius: {self.default_radius}m)', fontsize=14)
        print(f"Radius increased to {self.default_radius:.1f}m")
    
    def decrease_radius(self, event=None):
        """半径を0.1m減らす（最小0.1m）"""
        if self.default_radius > 0.1:
            self.default_radius -= 0.1
            self.update_status()
            self.ax.set_title(f'Robot Path - Click to Record Points (Section: {self.current_section}, Radius: {self.default_radius}m)', fontsize=14)
            print(f"Radius decreased to {self.default_radius:.1f}m")
        else:
            print("Radius is already at minimum (0.1m)")
    
    def next_section(self, event=None):
        """次のセクションに移動"""
        self.current_section += 1
        self.update_status()
        self.ax.set_title(f'Robot Path - Click to Record Points (Section: {self.current_section}, Radius: {self.default_radius}m)', fontsize=14)
        print(f"\n=== Moved to Section {self.current_section} ===")
        self.fig.canvas.draw()
    
    def update_status(self):
        """ステータステキストを更新"""
        self.status_text.set_text(f'Section: {self.current_section} | Points: {len(self.clicked_points)} | Radius: {self.default_radius:.1f}m')
        self.fig.canvas.draw()
    
    def update_clicked_points_plot(self):
        """クリックした点のプロットを更新（セクションごとに色分け）"""
        # 既存の円を削除
        for circle in self.radius_circles:
            circle.remove()
        self.radius_circles = []
        
        if self.clicked_points:
            x_coords = [p['x'] for p in self.clicked_points]
            y_coords = [p['y'] for p in self.clicked_points]
            sections = [p['section'] for p in self.clicked_points]
            
            # セクションごとに色を変えて表示
            self.clicked_scatter.set_offsets(list(zip(x_coords, y_coords)))
            self.clicked_scatter.set_array(sections)
            self.clicked_scatter.set_cmap('tab10')
            
            # 各点に半径の円を描画（セクションごとに色分け）
            import matplotlib.cm as cm
            cmap = cm.get_cmap('tab10')
            max_section = max(sections)
            
            for point in self.clicked_points:
                color = cmap((point['section'] - 1) % 10 / 10)
                circle = plt.Circle((point['x'], point['y']), 
                                   point['radius'], 
                                   color=color, 
                                   fill=False, 
                                   linewidth=2, 
                                   linestyle='--',
                                   alpha=0.5,
                                   zorder=9)
                self.ax.add_patch(circle)
                self.radius_circles.append(circle)
        else:
            self.clicked_scatter.set_offsets([])
        
        self.update_status()
        self.fig.canvas.draw()
    
    def on_click(self, event):
        """マウスクリック時に最も近い点の座標を記録"""
        # ボタンエリアのクリックは無視
        if event.inaxes != self.ax:
            return
        
        # 右クリックは無視
        if event.button != 1:  # 1 = 左クリック
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
        
        # クリックした点を記録（x, y, radius, sectionを含む）
        point_data = {
            'x': float(nearest_x),
            'y': float(nearest_y),
            'radius': float(self.default_radius),
            'section': int(self.current_section)
        }
        
        self.clicked_points.append(point_data)
        
        # 座標を表示
        coord_info = (f"Point #{len(self.clicked_points)} Added!\n"
                     f"Section: {self.current_section}\n"
                     f"X: {nearest_x:.3f} m\n"
                     f"Y: {nearest_y:.3f} m\n"
                     f"Radius: {self.default_radius:.1f} m\n"
                     f"Total Points: {len(self.clicked_points)}")
        
        self.coord_text.set_text(coord_info)
        
        # コンソールにも出力
        print(f"\n✓ Point #{len(self.clicked_points)} Added (Section {self.current_section})")
        print(f"  X: {nearest_x:.3f} m, Y: {nearest_y:.3f} m, Radius: {self.default_radius:.1f} m")
        
        # クリックした点のプロットを更新
        self.update_clicked_points_plot()
    
    def save_points(self, event=None):
        """クリックした点をCSVに保存"""
        if not self.clicked_points:
            print("No points to save!")
            self.coord_text.set_text("No points to save!")
            self.fig.canvas.draw()
            return
        
        try:
            # x, y, radius, section のDataFrameを作成
            df_output = pd.DataFrame(self.clicked_points)
            
            # 列の順序を保証
            df_output = df_output[['section', 'x', 'y', 'radius']]
            
            # CSVに保存（ヘッダー付き）
            df_output.to_csv(self.output_csv, index=False, float_format='%.6f')
            
            message = f"✓ Saved {len(self.clicked_points)} points\nto {os.path.basename(self.output_csv)}"
            print(f"\n{message.replace(chr(10), ' ')}")
            print(f"Full path: {self.output_csv}")
            self.coord_text.set_text(message)
            self.fig.canvas.draw()
            
        except Exception as e:
            error_msg = f"Error saving file: {e}"
            print(f"\n✗ {error_msg}")
            self.coord_text.set_text(error_msg)
            self.fig.canvas.draw()
    
    def clear_points(self, event=None):
        """すべてのクリック点をクリア"""
        if not self.clicked_points:
            print("No points to clear!")
            return
        
        num_points = len(self.clicked_points)
        self.clicked_points = []
        
        message = f"Cleared {num_points} points"
        print(f"\n{message}")
        self.coord_text.set_text(message)
        
        self.update_clicked_points_plot()
    
    def undo_last(self, event=None):
        """最後に追加した点を削除"""
        if not self.clicked_points:
            print("No points to undo!")
            self.coord_text.set_text("No points to undo!")
            self.fig.canvas.draw()
            return
        
        removed_point = self.clicked_points.pop()
        
        message = f"Removed point (Section {removed_point['section']})\nX: {removed_point['x']:.3f}, Y: {removed_point['y']:.3f}"
        print(f"\n✓ Removed point: Section={removed_point['section']}, X={removed_point['x']:.3f}, Y={removed_point['y']:.3f}")
        self.coord_text.set_text(message)
        
        self.update_clicked_points_plot()
    
    def show(self):
        """プロットを表示"""
        plt.show()
        
        # ウィンドウを閉じる時に自動保存するか確認
        if self.clicked_points:
            print(f"\n{len(self.clicked_points)} points recorded.")
            response = input(f"Save to {os.path.basename(self.output_csv)} before exit? (y/n): ")
            if response.lower() == 'y':
                self.save_points()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 plot_odometry_path.py <csv_filename> [downsample_factor] [output_csv] [default_radius]")
        print("Example: python3 plot_odometry_path.py odometry_path.csv 10 section_points.csv 0.5")
        print("         downsample_factor: Display every Nth point (default: 10)")
        print("         output_csv: Output filename for clicked points (default: clicked_points.csv)")
        print("         default_radius: Default radius for points in meters (default: 0.5)")
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
    
    # 出力CSVファイル名を取得（デフォルトはclicked_points.csv）
    output_csv = 'clicked_points.csv'
    if len(sys.argv) >= 4:
        output_csv = sys.argv[3]
    
    # デフォルト半径を取得（デフォルトは0.5m）
    default_radius = 0.5
    if len(sys.argv) >= 5:
        try:
            default_radius = float(sys.argv[4])
            if default_radius <= 0:
                print("Error: default_radius must be > 0")
                sys.exit(1)
        except ValueError:
            print("Error: default_radius must be a number")
            sys.exit(1)
    
    try:
        plotter = PathPlotter(csv_file, downsample_factor, output_csv, default_radius)
        plotter.show()
    except FileNotFoundError:
        print(f"Error: File '{csv_file}' not found!")
        sys.exit(1)
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)