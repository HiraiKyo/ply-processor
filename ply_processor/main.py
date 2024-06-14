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
plane_df = pd.DataFrame(None, columns=["a", "b", "c", "d", "calc_time"])
# 円柱データ
cylinder_df = (
    pd.DataFrame(
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
    ),
)

# 基準点からの円筒軸位置推定
estimated_df = pd.DataFrame(
    None,
    columns=[
        "distance_plane0",
        "distance_plane1",
        "distance_plane2",
        "distance_plane3",
        "distance_plane4",
        "radius",
        "calc_time",
    ],
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
        visualising_pcds.append(plane_inlier_cloud)

        # 板側面検出
        side_plane_inlier_cloud, side_plane_outlier_cloud, side_plane_model = (
            detect_plane(plane_outlier_cloud).ok_value
        )
        visualising_pcds.append(side_plane_inlier_cloud)
        # 板側面検出2
        side_plane2_inlier_cloud, side_plane2_outlier_cloud, side_plane2_model = (
            detect_plane(side_plane_outlier_cloud).ok_value
        )
        visualising_pcds.append(side_plane2_inlier_cloud)
        # 板側面検出3
        side_plane3_inlier_cloud, side_plane3_outlier_cloud, side_plane3_model = (
            detect_plane(side_plane2_outlier_cloud).ok_value
        )
        visualising_pcds.append(side_plane3_inlier_cloud)
        # 板側面検出4
        side_plane4_inlier_cloud, side_plane4_outlier_cloud, side_plane4_model = (
            detect_plane(side_plane3_outlier_cloud).ok_value
        )
        # 板側面検出5
        side_plane5_inlier_cloud, side_plane5_outlier_cloud, side_plane5_model = (
            detect_plane(side_plane4_outlier_cloud).ok_value
        )
        visualising_pcds.append(side_plane5_inlier_cloud)
        # 板側面検出6
        side_plane6_inlier_cloud, side_plane6_outlier_cloud, side_plane6_model = (
            detect_plane(side_plane5_outlier_cloud).ok_value
        )
        visualising_pcds.append(side_plane6_inlier_cloud)

        # 板内側切り出し
        clip3_inlier_cloud = side_plane6_outlier_cloud
        # clip_inlier_cloud, clip_outlier_cloud = clip_plane(
        #     side_plane2_outlier_cloud, plane_model
        # ).ok_value
        # visualising_pcds.append(clip_outlier_cloud)
        # clip2_inlier_cloud, clip2_outlier_cloud = clip_plane(
        #     clip_inlier_cloud, side_plane_model
        # ).ok_value
        # visualising_pcds.append(clip2_outlier_cloud)
        # clip3_inlier_cloud, clip3_outlier_cloud = clip_plane(
        #     clip2_inlier_cloud, side_plane2_model
        # ).ok_value
        # visualising_pcds.append(clip3_outlier_cloud)

        # 円筒軸推測
        cylinder_inlier_cloud, cylinder_outlier_cloud, cylinder_model = detect_cylinder(
            clip3_inlier_cloud, plane_model=plane_model
        ).ok_value
        visualising_pcds.append(cylinder_inlier_cloud)
        # 円筒フィッティング
        fit_cylinder_inlier_cloud, fit_cylinder_outlier_cloud, cylinder_model = (
            fit_cylinder(cylinder_inlier_cloud, axis=cylinder_model[:3]).ok_value
        )

        # 基準点からの円筒軸, 円半径位置算出
        d0 = point_plane_distance(cylinder_model[:3], plane_model)
        d1 = point_plane_distance(cylinder_model[:3], side_plane_model)
        d2 = point_plane_distance(cylinder_model[:3], side_plane2_model)
        d3 = point_plane_distance(cylinder_model[:3], side_plane3_model)
        d4 = point_plane_distance(cylinder_model[:3], side_plane4_model)
        r = cylinder_model[6]
        print(f"Estimated distances: ({d1:.2f}, {d2:.2f}, {d3:.2f}), radius: {r:.2f}")

        end = time.perf_counter()
        elapsed = "{:.1f}".format(end - start)

        print("Saving data...")
        # dataframeに格納
        plane_df.loc[i] = np.append(plane_model, elapsed)
        cylinder_df.loc[i] = np.append(cylinder_model, elapsed)
        estimated_df.loc[i] = np.append(d0, d1, d2, d3, d4, r, elapsed)

        # CSV保存
        plane_df.to_csv(f"{dir_name}/plane.csv")
        cylinder_df.to_csv(f"{dir_name}/cylinder.csv")
        estimated_df.to_csv(f"{dir_name}/estimated.csv")

        print("Visualizing...")
        filename = f"{dir_name}/{i}.png"

        capture_snapshot(
            vis,
            filename,
            cylinder_inlier_cloud.get_center(),
            visualising_pcds.append(cylinder_outlier_cloud),
        )
        # 円筒軸と半径からメッシュを作成
        if Config.MODE == "dev":
            plane_mesh = create_mesh_plane(plane_model, plane_inlier_cloud.get_center())
            cylinder_mesh = create_mesh_cylinder(
                cylinder_model, cylinder_inlier_cloud.get_center()
            )
            view_point_cloud(
                visualising_pcds.append(plane_mesh, cylinder_mesh), "最終結果"
            )
        i += 1

    vis.destroy_window()
    vis.close()


if __name__ == "__main__":
    main()
