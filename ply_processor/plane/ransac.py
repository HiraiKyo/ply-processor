import open3d as o3d
from result import Ok, Result
import pyransac3d as pyrsc
import numpy as np
from numpy.typing import NDArray
from ply_processor.config import Config


def detect_plane(
    pcd: o3d.geometry.PointCloud,
) -> Result[
    list[o3d.geometry.PointCloud, o3d.geometry.PointCloud, NDArray[np.float32]], str
]:
    """_summary_

    Args:
        pcd (o3d.geometry.PointCloud): _description_

    Returns:
        Result[ list[o3d.geometry.PointCloud, o3d.geometry.PointCloud, NDArray[np.float32]], str ]: _description_
    """
    points = np.asarray(pcd.points)
    plane = pyrsc.Plane()

    plane_model, inliers = plane.fit(points, 0.01, maxIteration=Config.MAX_ITERATION)

    [a, b, c, d] = plane_model
    print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

    # インライアの点を抽出して色を付ける
    inlier_cloud = pcd.select_by_index(inliers)
    inlier_cloud.paint_uniform_color([1.0, 0, 0])

    # 平面以外の点を抽出
    outlier_cloud = pcd.select_by_index(inliers, invert=True)

    # 可視化
    return Ok([inlier_cloud, outlier_cloud, plane_model])
