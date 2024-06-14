import numpy as np
import open3d as o3d
from result import Result, Ok
from ply_processor.config import Config
from ply_processor.geometry import get_rotation_matrix_from_vectors
from ply_processor.snapshot import view_point_cloud


def clip_plane(pcd_raw, plane_model) -> Result[
    list[
        o3d.geometry.PointCloud,
        o3d.geometry.PointCloud,
    ],
    str,
]:
    points_raw = np.asarray(pcd_raw.points)

    # 平面上の1点を原点、Z軸を平面の法線ベクトルとする座標系に変換
    transformation_matrix = np.eye(4)
    mean = np.mean(points_raw, axis=0)
    origin = np.array(
        [
            mean[0],
            mean[1],
            -(plane_model[3] + mean[0] * plane_model[0] + mean[1] * plane_model[1])
            / plane_model[2],
        ]
    )
    points = points_raw - origin

    if Config.MODE == "dev":
        coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        view_point_cloud([pcd, coordinate_frame], "クリッピング、平行移動後")

    transformation_matrix[:3, :3] = get_rotation_matrix_from_vectors(
        np.array([0, 0, 1]), plane_model[:3]
    )
    transformation_matrix_inv = np.linalg.inv(transformation_matrix)
    print(f"Transformation Matrix: {transformation_matrix}")

    # アフィン変換のために4x1に変換
    points = np.concatenate([points, np.ones((points.shape[0], 1))], axis=1)
    points = np.dot(transformation_matrix, points.T).T

    if Config.MODE == "dev":
        # 可視化
        coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[:, :3])
        view_point_cloud([pcd, coordinate_frame], "クリッピング、座標系変換後")

    # 平面上部の点群を抽出
    points_intp = np.where(points[:, 2] > 0.0)[0]
    points_intp_inv = np.where(points[:, 2] < 0.0)[0]
    if len(points_intp) < len(points_intp_inv):
        points_intp = points_intp_inv

    if len(points_intp) == 0:
        raise ValueError("No points has detected unexpectedly.")

    inlier_cloud = pcd_raw.select_by_index(points_intp)
    outlier_cloud = pcd_raw.select_by_index(points_intp, invert=True)
    outlier_cloud.paint_uniform_color([0, 0, 0])

    return Ok([inlier_cloud, outlier_cloud])
