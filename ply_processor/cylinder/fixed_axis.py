import open3d as o3d
import numpy as np
from numpy.typing import NDArray
from result import Ok, Result
from ply_processor.config import Config
from ply_processor.geometry import point_line_distance
from scipy.optimize import minimize

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
    # 重心を原点とする座標系に平行移動して処理速度向上
    n = len(points_raw)
    mean = sum(p for p in points_raw) / n
    points =  [p - mean for p in points_raw ]
    
    best_fit = None
    best_score = float("inf")
    
    # 最適化
    fitted = minimize(lambda x: eval_cylinder_model(x, axis, points), [0, 0, 0], method='Powell', tol=1e-6)
    best_score = fitted.fun
    best_fit = fitted
    
    # もとの座標系に戻す
    return axis, best_fit.x[:3] + mean, Config.MODEL["r"], best_score
    
    
def eval_cylinder_model(cylinder_model, axis, points) -> float:
    C = cylinder_model[:3]
    W = axis
    r = Config.MODEL["r"]
    
    distances = (point_line_distance(points, C, W) - r)**2
    return sum(distances) / len(points)