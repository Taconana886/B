"""相机-云台（IMU）手眼标定脚本（模块 B，B-4）。

改造说明（B-F6 / B-F8，2026-10-30）：
- 相机内参从 camParam yml 文件读取（cv2.FileStorage），不再写死；
- 棋盘格行列数、方格边长参数化（命令行），不再写死 (9,10)/0.027；
- 标定结果（R_c2i、T_c2i）连同内参自动写回新的 yml 文件，不再只 print；
- 姿态文件支持 3 列（yaw pitch roll，纯旋转假设，t_gimbals=0）或 6 列
  （yaw pitch roll tx ty tz，带平移）。

    约定：
    - euler_*.txt 每行一帧：yaw pitch roll（度，Z-X-Y 顺序之外的约定见
      `euler_angles_to_rotation_matrix` 注释），可选追加 tx ty tz（米）。
      欧拉角按"世界系 → 云台系"旋转（与 C++ 侧 Attitude 的 R_world2imu 语义一致）。
    - 图像 frame_*.png 与 euler_*.txt 按文件名中的标识符（`_` 后第二段）配对。
    - 输出 yml 字段：K、D、R_c2i、T_c2i，与 asset/camParam/ 现有文件格式一致，
      可直接作为 launch.cfg 的 camera_para 使用。旧文件不覆盖。

    手眼标定输入/输出约定（2026-10-30 用合成数据验证）：
    - OpenCV `calibrateHandEye` 的输入 `R_gripper2base` 是"云台系 → 世界系"，
      因此传入 euler 旋转矩阵的转置；`t_gripper2base` 为云台平移（纯旋转时全 0）。
    - 输出 `R_cam2gripper`/`t_cam2gripper` 为字面意义的"相机系 → 云台系"，
      与 yml 中 R_c2i/T_c2i 的定义（p_imu = R_c2i @ p_cam + T_c2i）一致，直接写出。
    - 旧脚本直接传入未转置的 R_world2gimbal 且只 print 不写回，两处均已修正；
      用旧脚本跑出的历史结果需按本节公式复核。

用法示例：
    python3 handeye_calibration.py \
        --data ./handeye_calibration_data/ \
        --intrinsics ../asset/camParam/cam_param_hik1_.yml \
        --board-w 9 --board-h 10 --square-size 0.027 \
        --out ../asset/camParam/cam_param_orin_20261030.yml
"""

import argparse
import glob
import os
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np


def euler_angles_to_rotation_matrix(theta_deg: np.ndarray) -> np.ndarray:
    """欧拉角（度）转旋转矩阵。

    注意：此处的旋转顺序 R = Ry(yaw) @ Rx(pitch) @ Rz(roll)，沿用旧脚本约定；
    与 C++ 侧 Attitude 的 Z-X-Y 顺序不同，取决于 euler_*.txt 录制时采用的
    约定。若换用新的姿态记录方式，需核对本函数与记录侧约定一致。
    """
    yaw, pitch, roll = np.radians(theta_deg)

    r_x = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, np.cos(pitch), -np.sin(pitch)],
            [0.0, np.sin(pitch), np.cos(pitch)],
        ]
    )
    r_y = np.array(
        [
            [np.cos(yaw), 0.0, np.sin(yaw)],
            [0.0, 1.0, 0.0],
            [-np.sin(yaw), 0.0, np.cos(yaw)],
        ]
    )
    r_z = np.array(
        [
            [np.cos(roll), -np.sin(roll), 0.0],
            [np.sin(roll), np.cos(roll), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    return r_y @ r_x @ r_z


def load_gimbal_poses(directory: str) -> Dict[str, Tuple[np.ndarray, np.ndarray]]:
    """读取 euler_*.txt，返回 {identifier: (R_gimbal, t_gimbal)}。

    每行 3 个数（yaw pitch roll，度）或 6 个数（追加 tx ty tz，米）。
    """
    poses: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
    for txt_path in sorted(glob.glob(os.path.join(directory, "euler_*.txt"))):
        identifier = os.path.splitext(os.path.basename(txt_path))[0].split("_")[1]
        try:
            with open(txt_path, "r", encoding="utf-8") as f:
                line = f.readline().strip()
            if not line:
                print(f"Warning: empty pose file {txt_path}")
                continue
            values = [float(v) for v in line.split()]
            if len(values) not in (3, 6):
                print(f"Warning: {txt_path} has {len(values)} values, expected 3 or 6; skipped")
                continue
            r_gimbal = euler_angles_to_rotation_matrix(np.array(values[:3]))
            t_gimbal = np.zeros(3, dtype=np.float64)
            if len(values) == 6:
                t_gimbal = np.array(values[3:6], dtype=np.float64)
            poses[identifier] = (r_gimbal, t_gimbal)
        except (OSError, ValueError) as e:
            print(f"Error reading {txt_path}: {e}")
    return poses


def load_intrinsics(path: str) -> Tuple[np.ndarray, np.ndarray]:
    """从 camParam yml 读取内参 K 与畸变 D。"""
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise RuntimeError(f"cannot open intrinsics file: {path}")
    k_node = fs.getNode("K")
    d_node = fs.getNode("D")
    if k_node.empty() or d_node.empty():
        raise RuntimeError(f"intrinsics file {path} must contain K and D")
    mtx = k_node.mat().astype(np.float64)
    dist = d_node.mat().astype(np.float64).reshape(1, -1)
    fs.release()
    return mtx, dist


def find_chessboard_corners(
    images: List[str],
    board_size: Tuple[int, int],
    square_size: float,
    mtx: np.ndarray,
    dist: np.ndarray,
    pose_dict: Dict[str, Tuple[np.ndarray, np.ndarray]],
) -> Tuple[List[np.ndarray], List[np.ndarray], List[np.ndarray], List[np.ndarray]]:
    """检测棋盘格并只返回同时存在云台姿态的帧。"""
    objp = np.zeros((board_size[0] * board_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0 : board_size[0], 0 : board_size[1]].T.reshape(-1, 2) * square_size

    r_gimbals: List[np.ndarray] = []
    t_gimbals: List[np.ndarray] = []
    rvecs: List[np.ndarray] = []
    tvecs: List[np.ndarray] = []

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    for img_path in images:
        identifier = os.path.splitext(os.path.basename(img_path))[0].split("_")[1]
        if identifier not in pose_dict:
            print(f"Warning: no pose file for {img_path}")
            continue

        img = cv2.imread(img_path)
        if img is None:
            print(f"Warning: could not read {img_path}")
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, board_size, None)

        if ret:
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            _, rvec, tvec = cv2.solvePnP(objp, corners2, mtx, dist)
            r_gimbal, t_gimbal = pose_dict[identifier]
            r_gimbals.append(r_gimbal)
            t_gimbals.append(t_gimbal)
            rvecs.append(rvec)
            tvecs.append(tvec)
        else:
            print(f"Chessboard not found in {img_path}")

    return r_gimbals, t_gimbals, rvecs, tvecs


def write_camera_param(
    out_path: str,
    mtx: np.ndarray,
    dist: np.ndarray,
    r_c2i: np.ndarray,
    t_c2i: np.ndarray,
) -> None:
    """把内参与外参写回 camParam 格式的 yml（新建文件，不覆盖旧文件）。"""
    if os.path.exists(out_path):
        raise RuntimeError(f"output file {out_path} already exists; refusing to overwrite")
    fs = cv2.FileStorage(out_path, cv2.FILE_STORAGE_WRITE)
    fs.write("K", mtx)
    fs.write("D", dist.reshape(1, -1))
    fs.write("R_c2i", r_c2i)
    fs.write("T_c2i", t_c2i.reshape(3, 1))
    fs.release()
    print(f"Wrote camera params to {out_path}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Eye-to-hand camera-gimbal calibration (module B)")
    parser.add_argument("--data", required=True, help="directory containing frame_*.png and euler_*.txt")
    parser.add_argument("--intrinsics", required=True, help="camParam yml file with K and D")
    parser.add_argument("--board-w", type=int, default=9, help="chessboard inner corners width")
    parser.add_argument("--board-h", type=int, default=10, help="chessboard inner corners height")
    parser.add_argument("--square-size", type=float, default=0.027, help="chessboard square size in meters")
    parser.add_argument("--out", default=None, help="output yml path (new file; default: <data>/cam_param_handeye.yml)")
    args = parser.parse_args()

    board_size = (args.board_w, args.board_h)
    if args.board_w <= 0 or args.board_h <= 0:
        print("Error: board size must be positive")
        return 1
    if args.square_size <= 0:
        print("Error: square size must be positive")
        return 1

    out_path = args.out or os.path.join(args.data, "cam_param_handeye.yml")

    mtx, dist = load_intrinsics(args.intrinsics)
    print(f"Loaded intrinsics from {args.intrinsics}")

    pose_dict = load_gimbal_poses(args.data)
    print(f"Loaded {len(pose_dict)} gimbal pose files")

    images = sorted(glob.glob(os.path.join(args.data, "frame_*.png")))
    if not images:
        print(f"Error: no frame_*.png found in {args.data}")
        return 1

    r_gimbals, t_gimbals, rvecs, tvecs = find_chessboard_corners(
        images, board_size, args.square_size, mtx, dist, pose_dict
    )

    print(f"Valid pose pairs: {len(r_gimbals)}")
    if len(r_gimbals) < 3:
        print("Error: need at least 3 valid pose pairs for hand-eye calibration!")
        return 1

    # Eye-to-hand：gripper = 云台，target = 棋盘格，base = 世界系（原点在云台旋转点）
    # r_gimbals/t_gimbals: 世界系 -> 云台系（euler 文件语义）
    # rvecs/tvecs: 世界系 -> 相机系（solvePnP 输出）
    # OpenCV 输入 R_gripper2base = 云台系 -> 世界系 = R_world2gimbal^T
    r_gripper2base = np.array([r_g.T for r_g in r_gimbals])
    t_gripper2base = np.array(t_gimbals)

    # 输出为相机系 -> 云台系，与 yml 的 R_c2i/T_c2i 语义一致，直接写出
    r_g2c, t_g2c = cv2.calibrateHandEye(
        r_gripper2base, t_gripper2base, np.array(rvecs), np.array(tvecs)
    )
    print("Gimbal to Camera Transformation (相机系 -> 云台系，即 yml 的 R_c2i/T_c2i):")
    print("Rotation:\n", r_g2c)
    print("Translation:\n", t_g2c)

    # 残差报告（B-4 验收④）
    # 平移：最小二乘解 t_target2world 与 t_cam2gripper，再与观测比较
    # 模型（相机系 -> 云台系为字面约定）：t_obs = R_g2c^T @ R_g @ t_target2world - R_g2c^T @ t_g2c
    a_mat = []
    b_vec = []
    for r_g, r_v, t_v in zip(r_gimbals, rvecs, tvecs):
        a_mat.append(np.hstack([r_g2c.T @ r_g, -r_g2c.T]))
        b_vec.append(t_v.flatten())
    ls_solution, _, _, _ = np.linalg.lstsq(np.vstack(a_mat), np.concatenate(b_vec), rcond=None)
    t_target2world_est = ls_solution[:3]
    t_g2c_est = ls_solution[3:]

    residuals_rot: List[float] = []
    residuals_tra: List[float] = []
    for i, (r_g, r_v, t_v) in enumerate(zip(r_gimbals, rvecs, tvecs)):
        r_obs = cv2.Rodrigues(r_v)[0]
        # 旋转残差：预测相对旋转 R_obs_i R_obs_j^T 与观测的夹角（j = i-1）
        if i > 0:
            r_obs_prev = cv2.Rodrigues(rvecs[i - 1])[0]
            r_pred_rel = r_g2c.T @ r_g @ r_gimbals[i - 1].T @ r_g2c
            residuals_rot.append(
                np.rad2deg(
                    np.linalg.norm(cv2.Rodrigues(r_pred_rel @ (r_obs @ r_obs_prev.T).T)[0])
                )
            )
        # 平移残差
        t_pred = r_g2c.T @ r_g @ t_target2world_est - r_g2c.T @ t_g2c_est
        residuals_tra.append(np.linalg.norm(t_pred - t_v.flatten()))
    mean_rot = float(np.mean(residuals_rot))
    mean_tra = float(np.mean(residuals_tra))
    print(f"Rotation residuals (deg): mean {mean_rot:.4f}, max {np.max(residuals_rot):.4f}")
    print(f"Translation residuals (m): mean {mean_tra:.4f}, max {np.max(residuals_tra):.4f}")
    print(
        f"Least-squares t_target2world: {t_target2world_est}, "
        f"t_g2c: {t_g2c_est.flatten()}"
    )

    # 残差过大说明姿态数据不足或配对错误，拒绝写出结果
    if mean_rot > 5.0 or mean_tra > 0.1:
        print(
            "Error: residuals too large (rot > 5 deg or trans > 0.1 m). "
            "Check pose pairing and use larger gimbal rotations."
        )
        return 1

    write_camera_param(out_path, mtx, dist, r_g2c, t_g2c)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
