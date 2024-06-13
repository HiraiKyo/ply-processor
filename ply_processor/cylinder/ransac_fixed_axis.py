import open3d as o3d
from result import Ok, Result
import numpy as np
from numpy.typing import NDArray
from ply_processor.config import Config
import random

from ply_processor.geometry import get_rotation_matrix_from_vectors, point_line_distance


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
    points = np.asarray(pcd.points)

    center, axis, r, inliers = fit(points, 0.1, maxIteration=Config.MAX_ITERATION)
    [x0, y0, z0] = center
    [a, b, c] = axis
    cylinder_model = np.concatenate([center, axis, np.asarray([r])])

    print(
        f"Cylinder axis equation: ({x0:.2f}, {y0:.2f}, {z0:.2f}) + t({a:.2f}, {b:.2f}, {c:.2f})"
    )
    print(f"Cylinder radius: {r}")

    # インライアの点を抽出して色を付ける
    inlier_cloud = pcd.select_by_index(inliers)
    inlier_cloud.paint_uniform_color([0, 1.0, 0])

    # 平面以外の点を抽出
    outlier_cloud = pcd.select_by_index(inliers, invert=True)

    # 可視化
    return Ok([inlier_cloud, outlier_cloud, cylinder_model])


def fit(points_raw, plane_model, thresh=0.2, maxIteration=1000):
    n_points = points_raw.shape[0]

    # 重心を原点、Z軸を平面の法線ベクトルとする座標系に変換
    mean = np.mean(points_raw, axis=0)
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, 3] = mean
    transformation_matrix[:3, :3] = get_rotation_matrix_from_vectors(np.array([0, 0, 0]), plane_model[3:6])
    points = np.dot(transformation_matrix, points_raw)
    # 軸は平面の法線ベクトル、半径は設定値で固定する
    c_fit = np.array([0, 0, 0])
    w_fit = plane_model[3:6]
    r_fit = Config.MODEL["r"]

    best_inliers = []

    for i in range(maxIteration):
        # パラメータをサンプリング
        c = 
        w = w_fit
        r = r_fit
        
        # 各点の中心軸との距離を算出し、閾値以下の点を抽出する
        distances = np.abs(point_line_distance(points, c, w) - r)
        pt_id_inliers = np.where(distances <= thresh)[0]

        # 点がより多く含まれる円筒面を見つける
        if len(pt_id_inliers) > len(best_inliers):
            best_inliers = pt_id_inliers
            c_fit = c

    return c_fit, w_fit, r_fit, best_inliers
