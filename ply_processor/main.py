import open3d as o3d
from ply_processor.plane.ransac import detect_plane
from ply_processor.cylinder.leastsq import detect_cylinder
import pandas as pd
import time
import os
from ply_processor.snapshot import (
    capture_snapshot,
    view_point_cloud,
    create_mesh_cylinder,
)
from ply_processor.config import Config
import numpy as np

pcd = o3d.io.read_point_cloud(Config.FILEPATH)

# データフレーム作成
# 平面の方程式
plane = pd.DataFrame(None, columns=["a", "b", "c", "d", "calc_time"])
# 円柱データ
cylinder = pd.DataFrame(
    None,
    columns=[
        "center_x",
        "center_y",
        "center_z",
        "axis_x",
        "axis_y",
        "axis_z",
        "radius",
        "calc_time",
    ],
)

# 日時でディレクトリを作成
dir_name = "out/" + time.strftime("%Y%m%d_%H%M%S")
os.mkdir(dir_name)


def main():
    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=False)
    for i in range(Config.LOOP):
        print("Processing loop:", i)

        # 実行時間を計測
        start = time.perf_counter()
        # フィッティング
        plane_inlier_cloud, plane_outlier_cloud, plane_model = detect_plane(
            pcd
        ).ok_value
        cylinder_inlier_cloud, cylinder_outlier_cloud, cylinder_model = detect_cylinder(
            plane_outlier_cloud, plane_model=plane_model
        ).ok_value

        end = time.perf_counter()
        elapsed = "{:.1f}".format(end - start)

        # dataframeに格納
        plane.loc[i] = np.append(plane_model, elapsed)
        cylinder.loc[i] = np.append(cylinder_model, elapsed)

        print("Visualizing...")
        filename = f"{dir_name}/{i}.png"
        cylinder_mesh = create_mesh_cylinder(cylinder_model)
        capture_snapshot(
            vis,
            filename,
            [
                plane_inlier_cloud,
                cylinder_inlier_cloud,
                cylinder_outlier_cloud,
                cylinder_mesh,
            ],
        )
        # 円筒軸と半径からメッシュを作成
        view_point_cloud(
            [
                plane_inlier_cloud,
                cylinder_inlier_cloud,
                cylinder_outlier_cloud,
                cylinder_mesh,
            ]
        )
        i += 1

    vis.destroy_window()
    vis.close()

    # CSV保存
    plane.to_csv(f"{dir_name}/plane.csv")
    cylinder.to_csv(f"{dir_name}/cylinder.csv")


if __name__ == "__main__":
    main()
