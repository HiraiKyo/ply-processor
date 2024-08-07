import tkinter as tk
from tkinter import filedialog

from .utils.config import ConfigBase

models = {
    "phi40": {"h_bottom": 10.0, "h_top": 30.0, "r": 40.0 / 2},
    "phi35": {"h_bottom": 10.0, "h_top": 30.0, "r": 35.0 / 2},
}

cam_presets = {
    "default": [-1, -1, 1],
    "front": [0, 0, -1],
    "top": [0, -1, 0],
    "right": [-1, 0, 0],
    "left": [1, 0, 0],
    "back": [0, 0, 1],
    "bottom": [0, 1, 0],
    "l45": [1, -1, 1],
    "r45": [-1, -1, 1],
    "l135": [1, -1, -1],
    "r135": [-1, -1, -1],
}


class Config(ConfigBase):
    SKIP_INITIAL_PLANES: int = 0
    MAX_PLANE_ITERATION: int = 20
    PLANE_POINTS_THRESHOLD: int = 500
    MAX_ITERATION: int = 1000
    LOOP: int = 1
    FILEPATH: str = "data/sample/sample.ply"
    INLIER_THRESHOLD: float = 1.0
    MODEL = models["phi35"]
    MODE = "prod"
    CAPTURE_ZOOM = 0.2
    CAM_FRONT = cam_presets["default"]

    def interactive_load_config(self):
        # FILEPATHを上書き変更するか質問する
        print(f'Do you want to change the file path? (Default: {self.FILEPATH}) [y/N]: ', end="")
        if input().lower() == "y":
            # ファイルブラウザを開く
            root = tk.Tk()
            root.withdraw()
            self.FILEPATH = filedialog.askopenfilename()
            print(f"FILEPATH: {self.FILEPATH}")

        # ノイズ対策、初期平面を何枚スキップするか選択
        print(f"Do you want to skip initial planes? (Default: {self.SKIP_INITIAL_PLANES}) [y/N]: ", end="")
        if input().lower() == "y":
            print("Enter the number of planes to skip: ", end="")
            skip_planes = input()
            if skip_planes.isnumeric() and int(skip_planes) >= 0:
                self.SKIP_INITIAL_PLANES = int(skip_planes)
            else:
                print("Invalid number. Using default value.")

        # 検出面の点数下限値を変更するか質問する
        print(f"Do you want to change the plane points threshold? (Default: {self.PLANE_POINTS_THRESHOLD}) [y/N]: ", end="")
        if input().lower() == "y":
            print("Enter the plane points threshold: ", end="")
            threshold = input()
            if threshold.isnumeric() and int(threshold) > 0:
                self.PLANE_POINTS_THRESHOLD = int(threshold)
            else:
                print("Invalid threshold. Using default value.")

        # MODELを上書きするか質問する
        print(f"Do you want to change the model? (Default: {self.MODEL}) [y/N]: ", end="")
        if input().lower() == "y":
            print("Available models:")
            for key, value in models.items():
                print(f"- {key}: {value}")
            print("Enter the model name: ", end="")
            model_name = input()
            if model_name in models:
                self.MODEL = models[model_name]
            else:
                print("Invalid model name. Using default model.")

        # カメラ位置をプリセットから選択するか質問する
        print(f"Do you want to change the camera position? (Default: {self.CAM_FRONT}) [y/N]: ", end="")
        if input().lower() == "y":
            print("Available camera presets:")
            for key, value in cam_presets.items():
                print(f"- {key}: {value}")
            print("Enter the camera position name: ", end="")
            cam_name = input()
            if cam_name in cam_presets:
                self.CAM_FRONT = cam_presets[cam_name]
            else:
                print("Invalid camera position name. Using default camera position.")

        # MODEを上書きするか質問する
        print("Do you want to change the mode? [y/N]: ", end="")
        if input().lower() == "y":
            print("Available modes:")
            print("- dev")
            print("- prod")
            print("Enter the mode name: ", end="")
            mode = input()
            if mode in ["dev", "prod"]:
                self.MODE = mode
            else:
                print("Invalid mode name. Using default mode.")

        # LOG_LEVELを上書きするか質問する
        print("Do you want to change the log level? [y/N]: ", end="")
        if input().lower() == "y":
            print("Available log levels:")
            print("- debug")
            print("- info")
            print("- warn")
            print("- error")
            print("Enter the log level name: ", end="")
            log_level = input()
            if log_level in ["debug", "info", "warn", "error"]:
                self.LOG_LEVEL = log_level
            else:
                print("Invalid log level name. Using default log level.")

        # OneShotモードかloopモードか
        print("Do you want to change the loop mode? [y/N]: ", end="")
        if input().lower() == "y":
            # ループ回数を設定
            print("Enter the loop count", end="")
            loop_count = input()
            if loop_count.isnumeric() and int(loop_count) > 0:
                self.LOOP = int(loop_count)
            else:
                print("Invalid loop count. Using one shot mode.")

        return self
