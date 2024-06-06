import tkinter as tk
from tkinter import filedialog
import os


def get_filepath_by_picker():
    root = tk.Tk()
    root.withdraw()
    fTyp = [("", "*.ply")]
    iDir = os.path.abspath(os.path.dirname(__file__))
    print("Select a ply file...")
    target_path = filedialog.askopenfilename(filetypes=fTyp, initialdir=iDir)
    return target_path
