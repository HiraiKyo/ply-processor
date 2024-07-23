import tkinter as tk
from tkinter import filedialog

from .utils.config import ConfigBase

models = {
    "phi40": {"h_bottom": 10.0, "h_top": 30.0, "r": 40.0 / 2},
    "phi35": {"h_bottom": 10.0, "h_top": 30.0, "r": 35.0 / 2},
}


class Config(ConfigBase):
    MAX_PLANE_ITERATION: int = 20
    PLANE_POINTS_THRESHOLD: int = 500
    MAX_ITERATION: int = 1000
    LOOP: int = 1
    FILEPATH: str = "data/sample/sample.ply"
    INLIER_THRESHOLD: float = 1.0
    MODEL = models["phi35"]
    MODE = "prod"
    CAPTURE_ZOOM = 0.2
    CAM_FRONT = [-1, -1, 1] # [0, 0, -1]

    def interactive_load_config(self):
        # FILEPATHを上書き変更するか質問する
        print("Do you want to change the file path? [y/N]: ", end="")
        if input().lower() == "y":
            # ファイルブラウザを開く
            root = tk.Tk()
            root.withdraw()
            self.FILEPATH = filedialog.askopenfilename(filetypes=[("PLY files", "*.ply")])
            print(f"FILEPATH: {self.FILEPATH}")

        # MODELを上書きするか質問する
        print("Do you want to change the model? [y/N]: ", end="")
        if input().lower() == "y":
            print("Available models:")
            for key, value in models.items():
                print(f"- {key}")
            print("Enter the model name: ", end="")
            model_name = input()
            if model_name in models:
                self.MODEL = models[model_name]
            else:
                print("Invalid model name. Using default model.")

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
