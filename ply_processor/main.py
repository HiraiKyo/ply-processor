import open3d as o3d
from ply_processor.clip.plane import clip_plane
from ply_processor.geometry import point_plane_distance
from ply_processor.plane.ransac import detect_plane
from ply_processor.cylinder.fixed_axis import detect_cylinder
from ply_processor.cylinder.leastsq import detect_cylinder as fit_cylinder
import pandas as pd
import time
import os
from ply_processor.snapshot import (
    capture_snapshot,
    view_point_cloud,
    create_mesh_cylinder,
    create_mesh_plane,
)
from ply_processor.config import Config
import numpy as np

# データフレーム作成
# 平面の方程式
columns = ["a", "b", "c", "d", "calc_time"]
plane_df = pd.DataFrame(None, columns=columns)
# 円柱データ
columns = ["x0", "y0", "z0", "a", "b", "c", "r", "calc_time", "points"]
cylinder_df = pd.DataFrame(None, columns=columns)
# 基準点からの円筒軸位置推定
columns = ["radius", "calc_time"]
columns += [f"distance_plane{i}" for i in range(Config.MAX_PLANE_ITERATION)]
estimated_df = pd.DataFrame(
    None,
    columns=columns,
)

# 日時でディレクトリを作成
dir_name = "out/" + time.strftime("%Y%m%d_%H%M%S")
os.mkdir(dir_name)


def main():
    pcd = o3d.io.read_point_cloud(Config.FILEPATH)

    # devモードでは点群生データを表示
    if Config.MODE == "dev":
        view_point_cloud([pcd])

    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=False)
    for i in range(Config.LOOP):
        print("Processing loop:", i)
        # 描画用点群格納リスト
        visualising_pcds = []

        # 実行時間を計測
        start = time.perf_counter()

        # 点群処理
        # 板底面検出
        plane_inlier_cloud, plane_outlier_cloud, plane_model = detect_plane(
            pcd
        ).ok_value

        # 板側面検出
        side_plane_inlier_clouds = []
        side_plane_outlier_clouds = []
        side_plane_models = []
        for j in range(Config.MAX_PLANE_ITERATION):
            last_cloud = (
                side_plane_outlier_clouds[-1]
                if side_plane_outlier_clouds
                else plane_outlier_cloud
            )
            side_plane_inlier_cloud, side_plane_outlier_cloud, side_plane_model = (
                detect_plane(last_cloud).ok_value
            )
            side_plane_inlier_clouds.append(side_plane_inlier_cloud)
            side_plane_outlier_clouds.append(side_plane_outlier_cloud)
            side_plane_models.append(side_plane_model)
            if len(side_plane_inlier_cloud.points) < Config.PLANE_POINTS_THRESHOLD:
                break
            j += 1

        visualising_pcds += side_plane_inlier_clouds

        clip_index = 2

        # 板内側切り出し
        clip_inlier_cloud, clip_outlier_cloud = clip_plane(
            side_plane_outlier_clouds[clip_index], plane_model
        ).ok_value
        clip2_inlier_cloud, clip2_outlier_cloud = clip_plane(
            clip_inlier_cloud, side_plane_models[0]
        ).ok_value
        clip3_inlier_cloud, clip3_outlier_cloud = clip_plane(
            clip2_inlier_cloud, side_plane_models[1]
        ).ok_value

        # 円筒軸推測
        cylinder_inlier_cloud, cylinder_outlier_cloud, cylinder_model = detect_cylinder(
            clip2_inlier_cloud, plane_model=plane_model
        ).ok_value
        visualising_pcds.append(cylinder_inlier_cloud)
        # # 円筒フィッティング
        # fit_cylinder_inlier_cloud, fit_cylinder_outlier_cloud, cylinder_model = (
        #     fit_cylinder(cylinder_inlier_cloud, axis=cylinder_model[:3]).ok_value
        # )

        # 基準点からの円筒軸, 円半径位置算出
        d = [
            point_plane_distance(cylinder_model[:3], plane)
            for plane in side_plane_models
        ]
        r = cylinder_model[6]
        print(f"Estimated distances: ({d}), radius: {r:.2f}")

        end = time.perf_counter()
        elapsed = "{:.1f}".format(end - start)

        print("Saving data...")
        # dataframeに格納
        plane_df.loc[i] = np.append(plane_model, elapsed)
        cylinder_df.loc[i] = np.concatenate(
            [cylinder_model, np.array([elapsed, len(cylinder_inlier_cloud.points)])]
        )
        newrow = np.array(
            [
                r,
                elapsed,
                *d,
                *[0 for _ in range(Config.MAX_PLANE_ITERATION - len(d))],
            ]
        )
        print(len(estimated_df.columns), len(newrow))
        estimated_df.loc[i] = newrow

        # CSV保存
        plane_df.to_csv(f"{dir_name}/plane.csv")
        cylinder_df.to_csv(f"{dir_name}/cylinder.csv")
        estimated_df.to_csv(f"{dir_name}/estimated.csv")

        print("Visualizing...")
        filename = f"{dir_name}/{i}.png"

        visualising_pcds.append(cylinder_outlier_cloud),
        capture_snapshot(
            vis,
            filename,
            cylinder_inlier_cloud.get_center(),
            visualising_pcds,
        )
        # 円筒軸と半径からメッシュを作成
        if Config.MODE == "dev":
            plane_mesh = create_mesh_plane(plane_model, plane_inlier_cloud.get_center())
            cylinder_mesh = create_mesh_cylinder(
                cylinder_model, cylinder_inlier_cloud.get_center()
            )
            visualising_pcds.append(plane_mesh)
            visualising_pcds.append(cylinder_mesh)
            view_point_cloud(visualising_pcds, "最終結果")
        i += 1

    vis.destroy_window()
    vis.close()


if __name__ == "__main__":
    main()
