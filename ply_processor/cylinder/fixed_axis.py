import open3d as o3d
import numpy as np
from numpy.typing import NDArray
from result import Ok, Result
from ply_processor.config import Config
from ply_processor.geometry import get_rotation_matrix_from_vectors, point_line_distance
from scipy.optimize import minimize

from ply_processor.snapshot import create_mesh_line, view_point_cloud


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
    # 前処理
    points = np.asarray(pcd.points)

    plane_model = kwargs["plane_model"]
    normal = plane_model[:3]

    # 円筒フィッティングロジック部分
    w_fit, C_fit, r_fit, fit_err = fit_fixed_axis(points, axis=normal)

    # 推定円筒方程式
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


def fit_fixed_axis(points_raw, axis):
    # Raw points
    coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points_raw)
    view_point_cloud([pcd, coordinate_frame], "座標系変換前")

    # 重心を原点、Z軸を平面の法線ベクトルとする座標系に変換
    transformation_matrix = np.eye(4)
    mean = np.mean(points_raw, axis=0)
    points = points_raw - mean
    line_set = create_mesh_line(
        np.concatenate([np.array([0, 0, 0]), np.array(axis) * 100])
    )
    pcd.points = o3d.utility.Vector3dVector(points)
    view_point_cloud([pcd, coordinate_frame, line_set], "平行移動後")
    transformation_matrix[:3, :3] = get_rotation_matrix_from_vectors(
        np.array([0, 0, 1]), axis
    )
    transformation_matrix_inv = np.linalg.inv(transformation_matrix)
    print(transformation_matrix)
    # アフィン変換のために4x1に変換
    points = np.concatenate([points, np.ones((points.shape[0], 1))], axis=1)
    points = np.dot(transformation_matrix, points.T).T

    # 可視化
    pcd.points = o3d.utility.Vector3dVector(points[:, :3])
    view_point_cloud([pcd, coordinate_frame], "座標系変換後")

    # 初期値
    c_fit = np.array([0, 0, 0, 0])
    w_fit = np.dot(transformation_matrix_inv[:3, :3], np.array(axis).T).T
    w_fit = np.append(w_fit, 1)
    r_fit = Config.MODEL["r"]
    fit_err = 0

    # 円筒側面の点群を設定値より抽出
    z = points[:, 2]
    points_intp = np.where(z > Config.MODEL["h_bottom"])[0]
    points_intp_inv = np.where(z < -Config.MODEL["h_bottom"])[0]
    if len(points_intp) < len(points_intp_inv):
        points_intp = points_intp_inv
    centers = []
    # 任意3点を選び、3点を通る円の方程式を求める
    for i in range(Config.MAX_ITERATION):
        indices = np.random.choice(points_intp, 3, replace=False)
        p0 = points[indices[0]]
        p1 = points[indices[1]]
        p2 = points[indices[2]]
        a, b, r = find_circle(p0, p1, p2)
        # 円半径が設定値プラマイ1以内の結果を収集する
        if r < Config.MODEL["r"] + 1 and r > Config.MODEL["r"] - 1:
            centers.append([a, b, 0, 0])  # z=0

    # 3点を通る円の中心のうち、設定値近傍の数を出力
    print(f"Number of circles: {len(centers)}, Iterations: {Config.MAX_ITERATION}")
    # 3点を通る円の中心の平均を求める
    c_fit = np.mean(centers, axis=0)

    # 逆変換前に中心軸描画
    print(f"Guess before converted: {c_fit[:3]}, {w_fit[:3]}, {r_fit}")
    line_set = create_mesh_line(np.concatenate([c_fit[:3], w_fit[:3] * 100]))
    view_point_cloud([pcd, line_set, coordinate_frame], "逆変換前、中心軸描画")

    # もとの座標系に戻す
    c_fit = np.dot(transformation_matrix_inv, c_fit.T).T
    w_fit = np.dot(transformation_matrix_inv, w_fit.T).T
    c_fit += mean
    return w_fit[:3], c_fit[:3], r_fit, fit_err


def find_circle(p0, p1, p2):
    x1, y1 = p0[:2]
    x2, y2 = p1[:2]
    x3, y3 = p2[:2]

    A = np.array([[x1, y1, 1], [x2, y2, 1], [x3, y3, 1]])
    D = np.linalg.det(A)

    Bx = np.array(
        [[x1**2 + y1**2, y1, 1], [x2**2 + y2**2, y2, 1], [x3**2 + y3**2, y3, 1]]
    )
    Dx = np.linalg.det(Bx)

    By = np.array(
        [[x1**2 + y1**2, x1, 1], [x2**2 + y2**2, x2, 1], [x3**2 + y3**2, x3, 1]]
    )
    Dy = np.linalg.det(By)

    a = Dx / (2 * D)
    b = -Dy / (2 * D)
    r = np.sqrt((a - x1) ** 2 + (b - y1) ** 2)

    return (a, b, r)