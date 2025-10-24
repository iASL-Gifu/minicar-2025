import cv2
import numpy as np
from pathlib import Path
import argparse
import shutil

def save_filtered(seq_dir, out_root, start_idx, end_idx, image_files, steers, speeds):
    seq_name = seq_dir.name
    out_dir = out_root / f"{seq_name}_s{start_idx:06d}_e{end_idx:06d}"
    out_img_dir = out_dir / "images"

    if out_dir.exists():
        print(f"[WARN] {out_dir} は既に存在します。上書きしません。")
        return

    out_img_dir.mkdir(parents=True, exist_ok=True)

    # 画像コピー
    for new_i, old_i in enumerate(range(start_idx, end_idx + 1)):
        shutil.copy(image_files[old_i], out_img_dir / f"{new_i:06d}.png")

    # numpy保存
    np.save(out_dir / "steers.npy", steers[start_idx:end_idx+1])
    np.save(out_dir / "speeds.npy", speeds[start_idx:end_idx+1])

    print(f"[SAVED] {out_dir} (frames={end_idx-start_idx+1})")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seq_dir", required=True, help="Path to extracted sequence dir")
    parser.add_argument("--outdir", required=True, help="Output root dir")
    args = parser.parse_args()

    seq_dir = Path(args.seq_dir).resolve()
    out_root = Path(args.outdir).resolve()

    images_dir = seq_dir / "images"
    image_files = sorted(images_dir.glob("*.png"))
    if not image_files:
        print("No images found in seq_dir/images")
        return

    steers = np.load(seq_dir / "steers.npy")
    speeds = np.load(seq_dir / "speeds.npy")
    total = len(image_files)

    idx = 0
    start_idx, end_idx = None, None

    cv2.namedWindow("preview", cv2.WINDOW_NORMAL)
    print("キー操作: ←/→:1移動 | a/d:10移動 | s:start | e:end | q:保存 | Esc:終了")

    while True:
        img = cv2.imread(str(image_files[idx]))
        vis = img.copy()

        text = f"{idx+1}/{total}"
        if start_idx is not None:
            text += f"  s={start_idx}"
        if end_idx is not None:
            text += f"  e={end_idx}"

        cv2.putText(vis, text, (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)
        cv2.imshow("preview", vis)

        key = cv2.waitKey(30) & 0xFF

        # ESC / Quit
        if key == 27:  # ESC
            print("キャンセル終了")
            break
        elif key == ord('q'):
            if start_idx is not None and end_idx is not None and start_idx < end_idx:
                save_filtered(seq_dir, out_root, start_idx, end_idx, image_files, steers, speeds)
            else:
                print("startかend未指定、または範囲不正")
            break

        # index 移動
        elif key in (81, 2424832):  # ←
            idx = max(0, idx - 1)
        elif key in (83, 2555904):  # →
            idx = min(total - 1, idx + 1)
        elif key == ord('a'):
            idx = max(0, idx - 10)
        elif key == ord('d'):
            idx = min(total - 1, idx + 10)

        # start/end 設定
        elif key == ord('s'):
            start_idx = idx
            print(f"start set: {start_idx}")
        elif key == ord('e'):
            end_idx = idx
            print(f"end set: {end_idx}")

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
