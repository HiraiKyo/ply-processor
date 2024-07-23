import os
import time
import pandas as pd
from ply_processor.config import Config


def create_log_frame():
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

    return plane_df, cylinder_df, estimated_df, dir_name
