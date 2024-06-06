import open3d as o3d
from result import Ok, Result
import numpy as np
from numpy.typing import NDArray
from ply_processor.config import Config
from scipy.optimize import leastsq
from pyransac3d import Cylinder
from ply_processor.geometry import point_line_distance
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

    w_fit, C_fit, r_fit, fit_err = fit(points)

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
    inliers = np.where(
        abs(point_line_distance(points, C_fit, w_fit) - r_fit) < Config.INLIER_THRESHOLD
    )
    inliers_cloud = pcd.select_by_index(inliers)
    inliers_cloud.paint_uniform_color([0, 1.0, 0])

    outliers = np.where(
        abs(point_line_distance(points, C_fit, w_fit) - r_fit)
        >= Config.INLIER_THRESHOLD
    )
    outliers_cloud = pcd.select_by_index(outliers)

    # 可視化
    return Ok([inliers_cloud, outliers_cloud, cylinder_model])
