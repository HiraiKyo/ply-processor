import open3d as o3d
from result import Result, Ok
import time
import numpy as np
from numpy.typing import NDArray
from ply_processor.geometry import get_rotation_matrix_from_vectors


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


def view_point_cloud(pcds: list):
    o3d.visualization.draw_geometries(
        pcds,
        window_name="Open3D",
        width=800,
        height=600,
        left=50,
        top=50,
        point_show_normal=False,
        mesh_show_wireframe=True,
    )
    return Ok(None)


def create_mesh_cylinder(cylinder_model: NDArray[np.float32]):
    """_summary_

    Args:
        cylinder_model (NDArray[np.float32]): _description_

    Returns:
        None: _description_
    """
    # 円筒メッシュの作成
    cylinder_mesh = o3d.geometry.TriangleMesh.create_cylinder(
        radius=cylinder_model[6],
        height=100,
        resolution=20,
        split=4,
    )

    p0 = cylinder_model[:3]
    v0 = np.asarray([0.0, 0.0, 1.0])
    v = cylinder_model[3:6]

    # 位置と姿勢の変換行列を作成
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, 3] = p0
    transformation_matrix[:3, :3] = get_rotation_matrix_from_vectors(v0, v)

    # 円筒を原点に移動
    cylinder_mesh.translate(-np.array(cylinder_mesh.get_center()))

    # 変換を適用
    cylinder_mesh.transform(transformation_matrix)

    return cylinder_mesh
