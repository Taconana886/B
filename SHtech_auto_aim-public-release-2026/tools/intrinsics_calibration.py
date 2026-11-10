"""相机内参标定脚本（模块 B，B-4 配套工具）。

用途：用棋盘格照片标定相机内参与畸变，输出与 `asset/camParam/` 现有文件兼容的
新 yml（K、D，可选保留模板中的 R_c2i/T_c2i），供 `tools/handeye_calibration.py`
与 `launch.cfg` 的 `camera_para` 使用。

用法示例：
    python3 tools/intrinsics_calibration.py \
        --data ~/calib/intrinsics \
        --board-w 9 --board-h 10 --square-size 0.025 \
        --template asset/camParam/cam_param_hik1_.yml \
        --out asset/camParam/cam_param_orin_20261030.yml

注意：
- `--board-w/--board-h` 是**内角点**数量（黑白格交点数），不是格子数；
- `--square-size` 是单个格子实际边长（米），用尺子量，误差会直接进入距离精度；
- 平均重投影误差 > 0.3 px 时默认**不写文件**（B-4 验收标准），`--force-write` 可强制；
- 输出文件已存在时拒绝覆盖（新设备/新日期单独建文件）。
"""

import argparse
import glob
import os
import sys
from typing import List, Optional, Tuple

import cv2
import numpy as np

IMAGE_PATTERNS = ("*.png", "*.jpg", "*.jpeg", "*.bmp")


def list_images(directory: str) -> List[str]:
    files: List[str] = []
    for pattern in IMAGE_PATTERNS:
        files.extend(glob.glob(os.path.join(directory, pattern)))
        files.extend(glob.glob(os.path.join(directory, pattern.upper())))
    return sorted(set(files))


def detect_corners(gray: np.ndarray, board_size: Tuple[int, int]) -> Optional[np.ndarray]:
    """检测棋盘格内角点，优先使用 SB 算法（对模糊/噪声更稳）。"""
    if hasattr(cv2, "findChessboardCornersSB"):
        try:
            found, corners = cv2.findChessboardCornersSB(
                gray, board_size, cv2.CALIB_CB_NORMALIZE_IMAGE
            )
        except cv2.error:
            found, corners = False, None
        if found:
            return corners

    found, corners = cv2.findChessboardCorners(gray, board_size, None)
    if not found:
        return None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    return cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)


def read_template_extrinsics(path: Optional[str]):
    """从模板 yml 读取 R_c2i/T_c2i；没有模板时返回单位阵/零平移。"""
    identity = np.eye(3, dtype=np.float64)
    zero = np.zeros((3, 1), dtype=np.float64)
    if not path:
        return identity, zero
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise RuntimeError(f"cannot open template file: {path}")
    r_node = fs.getNode("R_c2i")
    t_node = fs.getNode("T_c2i")
    r_c2i = r_node.mat().astype(np.float64) if not r_node.empty() else identity
    t_c2i = t_node.mat().astype(np.float64).reshape(3, 1) if not t_node.empty() else zero
    fs.release()
    return r_c2i, t_c2i


def write_camera_param(
    out_path: str,
    mtx: np.ndarray,
    dist: np.ndarray,
    r_c2i: np.ndarray,
    t_c2i: np.ndarray,
) -> None:
    if os.path.exists(out_path):
        raise RuntimeError(f"output file {out_path} already exists; refusing to overwrite")
    fs = cv2.FileStorage(out_path, cv2.FILE_STORAGE_WRITE)
    fs.write("K", mtx)
    fs.write("D", dist.reshape(1, -1))
    fs.write("R_c2i", r_c2i)
    fs.write("T_c2i", t_c2i.reshape(3, 1))
    fs.release()


def main() -> int:
    parser = argparse.ArgumentParser(description="Chessboard camera intrinsics calibration (module B)")
    parser.add_argument("--data", required=True, help="directory containing chessboard images")
    parser.add_argument("--board-w", type=int, default=9, help="chessboard inner corners (columns)")
    parser.add_argument("--board-h", type=int, default=10, help="chessboard inner corners (rows)")
    parser.add_argument("--square-size", type=float, required=True, help="chessboard square size in meters")
    parser.add_argument("--template", default=None, help="existing camParam yml; R_c2i/T_c2i are copied")
    parser.add_argument("--out", required=True, help="output yml path (new file; never overwritten)")
    parser.add_argument("--max-error", type=float, default=0.3, help="max mean reprojection error in px")
    parser.add_argument("--force-write", action="store_true", help="write yml even if error exceeds threshold")
    args = parser.parse_args()

    board_size = (args.board_w, args.board_h)
    if args.board_w <= 1 or args.board_h <= 1:
        print("Error: board size must be > 1 (inner corner counts)")
        return 1
    if args.square_size <= 0:
        print("Error: square size must be positive")
        return 1

    images = list_images(args.data)
    if not images:
        print(f"Error: no images found in {args.data}")
        return 1
    if len(images) < 10:
        print(f"Warning: only {len(images)} images; recommend >= 25 for a reliable calibration")

    objp = np.zeros((board_size[0] * board_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0 : board_size[0], 0 : board_size[1]].T.reshape(-1, 2) * args.square_size

    objpoints: List[np.ndarray] = []
    imgpoints: List[np.ndarray] = []
    used_images: List[str] = []
    image_size = None

    for path in images:
        img = cv2.imread(path)
        if img is None:
            print(f"Warning: could not read {path}")
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if image_size is None:
            image_size = gray.shape[::-1]
        elif gray.shape[::-1] != image_size:
            print(f"Warning: {path} size {gray.shape[::-1]} != {image_size}; skipped")
            continue
        corners = detect_corners(gray, board_size)
        if corners is None:
            print(f"Chessboard not found in {path}")
            continue
        objpoints.append(objp)
        imgpoints.append(corners)
        used_images.append(path)

    print(f"Detected chessboard in {len(objpoints)}/{len(images)} images")
    if len(objpoints) < 10:
        print("Error: need at least 10 valid images for calibration")
        return 1

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6)
    rms, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, image_size, None, None, criteria=criteria
    )

    per_image_errors: List[Tuple[float, str]] = []
    for i, (obj, img_pts, rvec, tvec) in enumerate(zip(objpoints, imgpoints, rvecs, tvecs)):
        projected, _ = cv2.projectPoints(obj, rvec, tvec, mtx, dist)
        err = np.linalg.norm(img_pts.reshape(-1, 2) - projected.reshape(-1, 2), axis=1)
        per_image_errors.append((float(np.mean(err)), used_images[i]))

    all_errors = [e for e, _ in per_image_errors]
    mean_error = float(np.mean(all_errors))
    p90_error = float(np.percentile(all_errors, 90))

    print(f"OpenCV RMS: {rms:.4f} px")
    print(f"Mean reprojection error: {mean_error:.4f} px | P90: {p90_error:.4f} px")
    worst = sorted(per_image_errors, reverse=True)[:5]
    print("Worst images:")
    for err, path in worst:
        print(f"  {err:8.4f} px  {os.path.basename(path)}")

    span_x = max(float(np.max(c[0, 0])) for c in imgpoints) - min(float(np.min(c[0, 0])) for c in imgpoints)
    span_y = max(float(np.max(c[0, 1])) for c in imgpoints) - min(float(np.min(c[0, 1])) for c in imgpoints)
    ratio_x, ratio_y = span_x / image_size[0], span_y / image_size[1]
    print(f"Corner coverage: {ratio_x * 100:.0f}% width, {ratio_y * 100:.0f}% height")
    if ratio_x < 0.6 or ratio_y < 0.6:
        print("Warning: coverage is small; add photos with the board near the image corners")

    print("K:\n", mtx)
    print("D:\n", dist.reshape(1, -1))

    if mean_error > args.max_error and not args.force_write:
        print(
            f"Error: mean error {mean_error:.4f} px > {args.max_error} px; "
            "yml NOT written. Remove blurry/outlier photos or use --force-write."
        )
        return 1

    r_c2i, t_c2i = read_template_extrinsics(args.template)
    try:
        write_camera_param(args.out, mtx, dist, r_c2i, t_c2i)
    except RuntimeError as e:
        print(f"Error: {e}")
        return 1
    print(f"Wrote camera params to {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
