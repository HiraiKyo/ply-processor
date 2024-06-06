import open3d as o3d
from result import Ok, Result
import numpy as np
import pyransac3d as pyrsc
from numpy.typing import NDArray
from ply_processor.config import Config


def detect_circle(
    pcd: o3d.geometry.PointCloud,
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

    Returns:
        Result[ list[ o3d.geometry.PointCloud, o3d.geometry.PointCloud, NDArray[np.float32]], str, ]: _description_
    """
    points = np.asarray(pcd.points)
    circle = pyrsc.Circle()

    center, axis, r, inliers = circle.fit(
        points, 0.1, maxIteration=Config.MAX_ITERATION
    )
    [x0, y0, z0] = center
    [a, b, c] = axis
    circle_model = np.concatenate([center, axis, np.asarray([r])])
    print(
        f"Circle axis equation: ({x0:.2f}, {y0:.2f}, {z0:.2f}) + t({a:.2f}, {b:.2f}, {c:.2f})"
    )
    print(f"Circle radius: {r}")

    # インライアの点を抽出して色を付ける
    inlier_cloud = pcd.select_by_index(inliers)
    inlier_cloud.paint_uniform_color([0, 0, 1.0])

    # 平面以外の点を抽出
    outlier_cloud = pcd.select_by_index(inliers, invert=True)

    # 可視化
    return Ok([inlier_cloud, outlier_cloud, circle_model])
