import open3d as o3d
from result import Ok, Result
import numpy as np
from numpy.typing import NDArray
from ply_processor.config import Config
import random
from ply_processor.snapshot import view_point_cloud
from ply_processor.utils.log import Logger

logger = Logger()


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
    plane = Plane()

    plane_model, inliers = plane.fit(
        points, thresh=Config.INLIER_THRESHOLD, maxIteration=Config.MAX_ITERATION
    )

    [a, b, c, d] = plane_model
    logger.debug(
        f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0, Points: {len(inliers)}",
    )

    # インライアの点を抽出して色を付ける
    inlier_cloud = pcd.select_by_index(inliers)
    inlier_cloud.paint_uniform_color([1.0, 0, 0])

    # 平面以外の点を抽出
    outlier_cloud = pcd.select_by_index(inliers, invert=True)

    if Config.MODE == "dev":
        coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10)
        view_point_cloud(
            [inlier_cloud, outlier_cloud, coordinate_frame], "座標系変換前"
        )

    # 可視化
    return Ok([inlier_cloud, outlier_cloud, plane_model])


class Plane:
    def __init__(self):
        self.inliers = []
        self.equation = []

    def fit(self, pts, thresh=0.01, minPoints=100, maxIteration=1000):
        n_points = pts.shape[0]
        best_eq = []
        best_inliers = []

        for it in range(maxIteration):
            id_samples = random.sample(range(0, n_points), 3)
            pt_samples = pts[id_samples]

            vecA = pt_samples[1, :] - pt_samples[0, :]
            vecB = pt_samples[2, :] - pt_samples[0, :]

            vecC = np.cross(vecA, vecB)

            vecC = vecC / np.linalg.norm(vecC)
            k = -np.sum(np.multiply(vecC, pt_samples[1, :]))
            plane_eq = [vecC[0], vecC[1], vecC[2], k]

            pt_id_inliers = []
            dist_pt = (
                plane_eq[0] * pts[:, 0]
                + plane_eq[1] * pts[:, 1]
                + plane_eq[2] * pts[:, 2]
                + plane_eq[3]
            ) / np.sqrt(plane_eq[0] ** 2 + plane_eq[1] ** 2 + plane_eq[2] ** 2)

            pt_id_inliers = np.where(np.abs(dist_pt) <= thresh)[0]
            if len(pt_id_inliers) > len(best_inliers):
                best_eq = plane_eq
                best_inliers = pt_id_inliers
            self.inliers = best_inliers
            self.equation = best_eq

        return self.equation, self.inliers
