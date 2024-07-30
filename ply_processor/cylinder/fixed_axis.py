import random
from typing import Tuple
import open3d as o3d
import numpy as np
from numpy.typing import NDArray
from result import Ok, Result
from ply_processor.config import Config
from ply_processor.geometry import get_rotation_matrix_from_vectors, point_line_distance

from ply_processor.snapshot import create_mesh_line, view_point_cloud
from ply_processor.utils.log import Logger

logger = Logger()


def detect_cylinder(
    pcd: o3d.geometry.PointCloud,
    plane_model: NDArray[np.float32],
) -> Result[
    Tuple[
        o3d.geometry.PointCloud,
        o3d.geometry.PointCloud,
        NDArray[np.float32],
    ],
    str,
]:
    points = np.asarray(pcd.points)

    # 円筒フィッティングロジック部分
    w_fit, C_fit, r_fit, fit_err = fit_fixed_axis(points, plane_model)

    # 推定円筒方程式
    cylinder_model = np.concatenate([C_fit, w_fit, np.array([r_fit])])

    x0, y0, z0 = C_fit
    a, b, c = w_fit
    r = r_fit
    logger.debug(
        f"Cylinder axis equation: ({x0:.2f}, {y0:.2f}, {z0:.2f}) + t({a:.2f}, {b:.2f}, {c:.2f})"
    )
    logger.debug(f"Cylinder radius: {r}")

    # PCDに色付け
    # 推定した円筒モデル近傍の点を抽出
    # インデックスを取得したい
    distances = abs(point_line_distance(points, C_fit, w_fit) - r_fit)
    inliers = np.where(distances < Config.INLIER_THRESHOLD)[0]
    inliers_cloud = pcd.select_by_index(inliers)
    inliers_cloud.paint_uniform_color([0, 1.0, 0])

    logger.debug(f"Points in cylinder surface: {len(inliers_cloud.points)}")
    outliers = np.where(distances >= Config.INLIER_THRESHOLD)[0]
    outliers_cloud = pcd.select_by_index(outliers)

    return Ok((inliers_cloud, outliers_cloud, cylinder_model))


def fit_fixed_axis(points_raw, plane_model):
    if Config.MODE == "dev":
        # Raw points
        coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points_raw)
        view_point_cloud([pcd, coordinate_frame], "円筒軸算出 座標系変換前")

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
        line_set = create_mesh_line(
            np.concatenate([np.array([0, 0, 0]), np.array(plane_model[:3]) * 100])
        )
        coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        view_point_cloud([pcd, coordinate_frame, line_set], "円筒軸算出 平行移動後")

    transformation_matrix[:3, :3] = get_rotation_matrix_from_vectors(
        np.array([0, 0, 1]), plane_model[:3]
    )
    transformation_matrix_inv = np.linalg.inv(transformation_matrix)
    # アフィン変換のために4x1に変換
    points = np.concatenate([points, np.ones((points.shape[0], 1))], axis=1)
    points = np.dot(transformation_matrix, points.T).T

    if Config.MODE == "dev":
        # 可視化
        coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[:, :3])
        view_point_cloud([pcd, coordinate_frame], "円筒軸算出 座標系変換後")

    # 初期値
    c_fit = np.array([0, 0, 0, 1])
    w_fit = np.array([0, 0, 1, 1])
    r_fit = Config.MODEL["r"]
    fit_err = 0

    # 円筒側面の点群を設定値より抽出
    points_intp = np.where(points[:, 2] > Config.MODEL["h_bottom"])[0]
    points_intp_inv = np.where(points[:, 2] < -Config.MODEL["h_bottom"])[0]
    if len(points_intp) < len(points_intp_inv):
        points_intp = points_intp_inv
        points = points[points_intp]
        points_intp = np.where(points[:, 2] > -Config.MODEL["h_top"])[0]
    else:
        points = points[points_intp]
        points_intp = np.where(points[:, 2] < Config.MODEL["h_top"])[0]

    if len(points_intp) == 0:
        raise ValueError("No points in the cylinder side")

    if Config.MODE == "dev":
        pcd = o3d.geometry.PointCloud()
        tmp = points[points_intp]
        pcd.points = o3d.utility.Vector3dVector(tmp[:, :3])
        view_point_cloud([pcd], "円筒軸算出 フィッティング用切り出し部分の点群")

    (c_fit, _, _, _) = fit(
        points,
        points_intp,
        np.asarray([0.0, 0.0, 1.0, 0.0]),
        thresh=0.1,
        maxIteration=Config.MAX_ITERATION,
    )

    if Config.MODE == "dev":
        # 逆変換前に中心軸描画
        coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=10)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[:, :3])
        logger.debug(f"Guess before converted: {c_fit[:3]}, {w_fit[:3]}, {r_fit}")
        line_set = create_mesh_line(np.concatenate([c_fit[:3], w_fit[:3] * 100]))
        view_point_cloud([pcd, line_set, coordinate_frame], "円筒軸算出 逆変換前、中心軸描画")

    # もとの座標系に戻す
    c_fit = np.dot(transformation_matrix_inv, c_fit.T).T
    w_fit = np.dot(transformation_matrix_inv, w_fit.T).T
    c_fit = c_fit[:3] + origin
    w_fit = w_fit[:3]

    return w_fit, c_fit, r_fit, fit_err


def fit(points_raw, pointers, plane_model, thresh=0.2, maxIteration=1000):
    points = points_raw

    # 軸は平面の法線ベクトル、半径は設定値で固定する
    c_fit = np.array([0.0, 0.0, 0.0, 1.0])
    w_fit = plane_model[:3]
    r_fit = Config.MODEL["r"]

    best_inliers = []

    for i in range(maxIteration):
        # サンプリング
        # 任意2点を選び、2点を通る半径rの円の方程式を求める
        indices = np.random.choice(pointers, 1, replace=False)
        p0 = points[indices[0]]
        # 残りの任意点は、直径以内の距離から選択する
        p1 = find_in_distance(p0, points, Config.MODEL["r"] * 2)
        cs = find_circle_with_radius(p0, p1, Config.MODEL["r"])

        c = np.concatenate([cs[random.randint(0, 1)], np.asarray([0.0, 1.0])])
        w = w_fit
        r = r_fit

        # 各点の中心軸との距離を算出し、閾値以下の点を抽出する
        distances = np.abs(point_line_distance(points[:, :3], c[:3], w) - r)
        pt_id_inliers = np.where(distances <= thresh)[0]

        # 点がより多く含まれる円筒面を見つける
        if len(pt_id_inliers) > len(best_inliers):
            best_inliers = pt_id_inliers
            c_fit = c

    return c_fit, w_fit, r_fit, best_inliers


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


# 2点を通り半径rの円の中心を求める
def find_circle_with_radius(p0, p1, r):
    # 中点
    m = (p0[:2] + p1[:2]) / 2
    d = np.linalg.norm(p0[:2] - p1[:2])
    if d < 1e-6:
        raise ValueError("Two points are too close to each other.")

    if d / 2 > r:
        raise ValueError("The distance between two points is larger than the diameter.")

    # 弦の単位ベクトル
    pd = (p1[:2] - p0[:2]) / d
    # 垂直二等分線の単位ベクトル
    n = np.array([-pd[1], pd[0]])
    # 弦の距離
    h = np.sqrt(r**2 - (d / 2) ** 2)
    # 中心
    c0 = m + h * n
    c1 = m - h * n
    return (c0, c1)


def find_in_distance(p0, points, distance, min_distance=0.0):
    distances = np.linalg.norm(points - p0, axis=1)
    inliers = np.where(distances < distance)[0]
    if len(inliers) == 0:
        raise ValueError("No inliers found in the distance.")

    indices = np.random.choice(inliers, 1, replace=False)
    return points[indices[0]]
