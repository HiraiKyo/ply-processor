import os
import sys
from PyQt6.QtWidgets import QApplication, QSizePolicy, QSpacerItem, QWidget, QPushButton, QVBoxLayout, QHBoxLayout, QLabel, QGridLayout
from PyQt6.QtGui import QPixmap
import pandas as pd

grid_row = 4

def open_gui(dir_name: str):
    """
    Open Result Viewer with PyQt6
    """
    # dir_nameのフォルダを展開し、その中にあるCSVとPNGを読み込む
    # ファイル名の規則は以下の通り
    # - plane.csv
    # - cylinder.csv
    # - estimated.csv
    # - {i}/pin.png
    # - {i}/plane_{j}.png
    image_dir_name = f'{dir_name}/0'
    image_list = os.listdir(image_dir_name)
    image_planes = filter(lambda x: x.startswith('plane_'), image_list)
    image_paths = {
        "pin": f'{image_dir_name}/pin.png',
        "plane": f'{image_dir_name}/plane.png',
        "planes": [f'{image_dir_name}/{x}' for x in image_planes]
    }
    df = pd.read_csv(f'{dir_name}/estimated.csv')
    radius = df['radius']
    calc_time = df['calc_time']
    distances = df.filter(like='distance_plane').values[0]

    print(image_paths)
    # ファイルを読み込む
    # GUIに表示
    qAp = QApplication(sys.argv)
    viewer = Viewer(radius, calc_time, distances, image_paths)
    viewer.show()
    qAp.exec()

    return None


window_size = (100, 100, 1440, 900)

# View Root Widget
class Viewer(QWidget):
    def __init__(self, radius, calc_time, distances, image_paths):
        super().__init__()
        self.radius = radius
        self.calc_time = calc_time
        self.distances = distances
        self.image_paths = image_paths
        self.initUI()
        self.show()

    def initUI(self):
        self.setWindowTitle('Result Viewer')
        self.setGeometry(*window_size)

        v = QVBoxLayout()
        self.setLayout(v)

        # 測定基本情報を表示
        hbox1 = QHBoxLayout()
        v.addLayout(hbox1)
        hbox1.addWidget(ImageWithLabel(self.image_paths["pin"], 'Pin'))
        hbox1.addWidget(ImageWithLabel(self.image_paths["plane"], 'Base Plane'))
        hbox1.addWidget(ParamTable(self.radius, self.calc_time, self.distances))
        hbox1.addItem(HorizontalSpacer())

        # 横並びで画像と各面からの距離を表示
        hbox2 = QGridLayout()
        v.addLayout(hbox2)
        for path in self.image_paths["planes"]:
            # pathをパースして、plane_{i}.pngのiを取得
            i = int(path.split('/')[-1].split('_')[-1].split('.')[0])
            hbox2.addWidget(ImageWithLabel(path, f'Plane {i}: {self.distances[i]:.2f}mm'), i // grid_row, i % grid_row)

        # レイアウト調整用スペーサー
        spacer = Spacer()
        v.addItem(spacer)

        hbox3 = QHBoxLayout()
        hbox3.addItem(HorizontalSpacer())
        button = QPushButton("Close", self)
        button.clicked.connect(self.close)
        hbox3.addWidget(button)
        v.addLayout(hbox3)

# Components
class ImageWithLabel(QWidget):
    def __init__(self, image_path: str, label: str):
        super().__init__()

        self.initUI(image_path, label)

    def initUI(self, image_path: str, text: str):
        v = QVBoxLayout()
        self.setLayout(v)
        label = QLabel()
        label.setText(text)
        img = QLabel()
        pix = QPixmap(image_path)
        pix = pix.scaledToWidth(360)
        img.setPixmap(pix)

        v.addWidget(label)
        v.addWidget(img)
        pass

class ParamTable(QWidget):
    def __init__(self, radius, calc_time, distances):
        super().__init__()

        self.initUI(radius, calc_time, distances)

    def initUI(self, radius, calc_time, distances):
        v = QVBoxLayout()
        self.setLayout(v)
        table = QGridLayout()
        v.addLayout(table)
        table.addWidget(QLabel('Radius'), 0, 0)
        table.addWidget(QLabel('Calc Time'), 0, 1)
        for i, (r, t, d) in enumerate(zip(radius, calc_time, distances)):
            table.addWidget(QLabel(f'{r:.2f}mm'), i + 1, 0)
            table.addWidget(QLabel(f'{t:.2f}s'), i + 1, 1)

        # レイアウト調整用スペーサー
        spacer = Spacer()
        v.addItem(spacer)

        pass

class Spacer(QSpacerItem):
    def __init__(self):
        super().__init__(0, 0, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)
        pass

class HorizontalSpacer(QSpacerItem):
    def __init__(self):
        super().__init__(0, 0, QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Minimum)
        pass