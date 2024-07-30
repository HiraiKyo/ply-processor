import os
import open3d as o3d
from ply_processor.clip.plane import clip_plane
from ply_processor.geometry import point_plane_distance
from ply_processor.gui import open_gui
from ply_processor.log import create_log_frame
from ply_processor.utils.log import Logger

from ply_processor.plane.ransac import detect_plane
from ply_processor.cylinder.fixed_axis import detect_cylinder
import time
from ply_processor.snapshot import (
    capture_snapshot,
    view_point_cloud,
    create_mesh_cylinder,
    create_mesh_plane,
)
from ply_processor.config import Config
import numpy as np


def main():
    # 初期セットアップ
    Config.interactive_load_config(Config)
    plane_df, cylinder_df, estimated_df, dir_name = create_log_frame()
    logger = Logger()
    logger.init(Config)
    try:
        vis = o3d.visualization.Visualizer()
        vis.create_window(visible=False)

        # ロード
        pcd = o3d.io.read_point_cloud(Config.FILEPATH)
        pcd.paint_uniform_color([0.5, 0.5, 0.5]) # 読み込んだ時の色をすべて灰色にする
        logger.info(f"Loaded ply file: {Config.FILEPATH}")

        view_point_cloud([pcd])

        for i in range(Config.LOOP):
            logger.debug(f"Processing loop: {i}")
            # 描画用点群格納リスト
            visualising_pcds = []

            # 実行時間を計測
            start = time.perf_counter()

            # 点群処理
            # ノイズ面スキップ
            skipped_plane_outlier_cloud = pcd
            for j in range(Config.SKIP_INITIAL_PLANES):
                logger.debug(f"Skipping initial plane {j}...")
                skipped_plane_inlier_cloud, skipped_plane_outlier_cloud, plane_model = detect_plane(
                    skipped_plane_outlier_cloud
                ).ok_value
                visualising_pcds.append(skipped_plane_inlier_cloud)

            # 板底面検出
            logger.debug("Detecting base plane...")
            plane_inlier_cloud, plane_outlier_cloud, plane_model = detect_plane(
                skipped_plane_outlier_cloud
            ).ok_value

            # 板側面検出
            side_plane_inlier_clouds = []
            side_plane_outlier_clouds = []
            side_plane_models = []
            for j in range(Config.MAX_PLANE_ITERATION):
                logger.debug(f"Detecting side plane {j}...")
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
            logger.debug("Clipping plane...")
            logger.debug(f"Clipping index: {clip_index}")
            clip_inlier_cloud, clip_outlier_cloud = clip_plane(
                side_plane_outlier_clouds[clip_index], plane_model
            ).ok_value
            logger.debug(f"Clipping index: {clip_index + 1}")
            clip2_inlier_cloud, clip2_outlier_cloud = clip_plane(
                clip_inlier_cloud, side_plane_models[0]
            ).ok_value
            logger.debug(f"Clipping index: {clip_index + 2}")
            clip3_inlier_cloud, clip3_outlier_cloud = clip_plane(
                clip2_inlier_cloud, side_plane_models[1]
            ).ok_value

            # 円筒軸推測
            logger.debug("Detecting cylinder...")
            cylinder_inlier_cloud, cylinder_outlier_cloud, cylinder_model = detect_cylinder(
                clip2_inlier_cloud, plane_model=plane_model
            ).ok_value
            visualising_pcds.append(cylinder_inlier_cloud)

            # 基準点からの円筒軸, 円半径位置算出
            d = [
                point_plane_distance(cylinder_model[:3], plane)
                for plane in side_plane_models
            ]
            r = cylinder_model[6]
            logger.info(f"Estimated distances: ({d}), radius: {r:.2f}")

            end = time.perf_counter()
            elapsed = "{:.1f}".format(end - start)

            logger.debug("Storing data...")
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
            estimated_df.loc[i] = newrow

            # スナップショット撮影
            logger.debug("Taking a snapshot...")
            os.mkdir(f"{dir_name}/{i}")
            # ピン撮影
            filename = f"{dir_name}/{i}/pin.png"
            capture_snapshot(
                vis,
                filename,
                [pcd, cylinder_inlier_cloud],
                Config.CAM_FRONT,  # FIXME: カメラ位置は調整が必要
                cylinder_inlier_cloud.get_center(),
                cylinder_model[:3],
            )

            # 基準面撮影
            filename = f"{dir_name}/{i}/plane.png"
            capture_snapshot(
                vis,
                filename,
                [pcd, plane_inlier_cloud],
                Config.CAM_FRONT,  # FIXME: カメラ位置は調整が必要
                cylinder_inlier_cloud.get_center(),
                cylinder_model[:3],
            )

            # 他検出面撮影
            for j, cloud in enumerate(side_plane_inlier_clouds):
                filename = f"{dir_name}/{i}/plane_{j}.png"
                capture_snapshot(
                    vis,
                    filename,
                    [pcd, cloud],
                    Config.CAM_FRONT,  # FIXME: カメラ位置は調整が必要
                    cylinder_inlier_cloud.get_center(),
                    cylinder_model[:3],
                )

            # フィット度合の目視検証のため、基準面メッシュと円筒メッシュを表示
            if Config.MODE == "dev":
                plane_mesh = create_mesh_plane(plane_model, plane_inlier_cloud.get_center())
                cylinder_mesh = create_mesh_cylinder(
                    cylinder_model, cylinder_inlier_cloud.get_center()
                )
                pcds = [pcd, plane_mesh, cylinder_mesh]
                view_point_cloud(pcds, "最終結果")

            i += 1

        # CSV保存
        plane_df.to_csv(f"{dir_name}/plane.csv")
        logger.info(f"Saved plane measurements to {dir_name}/plane.csv")
        cylinder_df.to_csv(f"{dir_name}/cylinder.csv")
        logger.info(f"Saved cylinder measurements to {dir_name}/cylinder.csv")
        estimated_df.to_csv(f"{dir_name}/estimated.csv")
        logger.info(f"Saved estimated measurements to {dir_name}/estimated.csv")
        vis.destroy_window()
        vis.close()

        # 結果画面を表示
        logger.info("Opening Result Viewer...")
        open_gui(dir_name)
    except Exception as e:
        print(e)
        logger.error(e)
    finally:
        # ログ保存
        logger.export(dir_name)


if __name__ == "__main__":
    main()
