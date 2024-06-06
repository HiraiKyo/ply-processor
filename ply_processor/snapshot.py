import open3d as o3d
from result import Result, Ok
import time
import numpy as np


def capture_snapshot(
    vis,
    filename: str,
    pcds: list,
) -> Result[None, str]:
    vis.clear_geometries()

    for pcd in pcds:
        vis.add_geometry(pcd)

    opt = vis.get_render_option()
    opt.show_coordinate_frame = True
    opt.mesh_show_back_face = True
    opt.mesh_show_wireframe = True  # メッシュのワイヤーフレームを表示
    opt.background_color = np.asarray([0.8, 0.8, 0.8])  # 背景色を設定

    # Zoom, front, lookat, upの設定
    ctr = vis.get_view_control()
    ctr.set_zoom(0.8)
    ctr.set_front([-0.4999, -0.1659, -0.8499])
    ctr.set_lookat([2.1813, 2.0619, 2.0999])
    ctr.set_up([0.1204, -0.9852, 0.1215])

    for pcd in pcds:
        vis.update_geometry(pcd)

    # 画像保存
    vis.poll_events()
    vis.update_renderer()
    time.sleep(1)
    vis.capture_screen_image(filename, do_render=True)

    return Ok(None)
