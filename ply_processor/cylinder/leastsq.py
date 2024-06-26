import open3d as o3d
from result import Ok, Result
import numpy as np
from numpy.typing import NDArray
from ply_processor.config import Config
from scipy.optimize import leastsq
from ply_processor.geometry import normalize, point_line_distance
from cylinder_fitting import fit


def detect_cylinder(
    pcd: o3d.geometry.PointCloud,
    **kwargs,
) -> Result[
    list[
        o3d.geometry.PointCloud,
        o3d.geometry.PointCloud,
        NDArray[np.float32],
    ],
    str,
]:
    """_summary_

    Args:
        pcd (o3d.geometry.PointCloud): _description_
        **kwargs: _description_

    Returns:
        Result[ list[ o3d.geometry.PointCloud, o3d.geometry.PointCloud, NDArray[np.float32]], str, ]: _description_
    """

    # 前処理
    points = np.asarray(pcd.points)
    guess_angles = None

    # 初期軸の設定
    axis = kwargs.get("axis")
    if axis is not None:
        axis = normalize(axis)
        r = np.linalg.norm(axis)
        # 法線ベクトルを極座標変換
        theta = np.arccos(axis[2] / r)
        phi = np.arctan2(axis[1], axis[0])
        guess_angles = [(theta, phi)]

    w_fit, C_fit, r_fit, fit_err = fit(points, guess_angles=guess_angles)

    cylinder_model = np.concatenate([C_fit, w_fit, np.array([r_fit])])

    x0, y0, z0 = C_fit
    a, b, c = w_fit
    r = r_fit
    print(
        f"Cylinder axis equation: ({x0:.2f}, {y0:.2f}, {z0:.2f}) + t({a:.2f}, {b:.2f}, {c:.2f})"
    )
    print(f"Cylinder radius: {r}")

    # PCDに色付け
    # 推定した円筒モデル近傍の点を抽出
    # インデックスを取得したい
    distances = abs(point_line_distance(points, C_fit, w_fit) - r_fit)
    inliers = np.where(distances < Config.INLIER_THRESHOLD)[0]
    inliers_cloud = pcd.select_by_index(inliers)
    inliers_cloud.paint_uniform_color([0, 1.0, 0])

    outliers = np.where(distances >= Config.INLIER_THRESHOLD)[0]
    outliers_cloud = pcd.select_by_index(outliers)

    # 可視化
    return Ok([inliers_cloud, outliers_cloud, cylinder_model])
