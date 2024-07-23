# ply-processor

## Installation

### Install poetry

poetry をインストール後、

```sh
poetry config virtualenvs.in-project true
```

### Install Packages

```sh
sudo apt install python3.*-tk tk-dev
poetry install
```

## Run

```sh
poetry run task main
```

## Config

### インタラクティブな設定値

| Param     | Values                               | Default                    | Misc.                                                           |
| --------- | ------------------------------------ | -------------------------- | --------------------------------------------------------------- |
| FILEPATH  | str                                  | `"data/sample/sample.ply"` | `data`フォルダに`ply`ファイルを入れる                           |
| LOOP      | Positive Int                         | `1`                        | 基本は OneShot なので`1`, 精度検証で反復する場合は増やす        |
| MODEL     | `models["phi35"]`, `models["phi40"]` | `phi35`                    | φ35 か φ40 のピンどちらを検証するか                             |
| MODE      | `dev`, `prod`                        | `prod`                     | `dev`は途中に点群表示して経過が見れる。`prod`は完了まで一気通貫 |
| LOG_LEVEL | `debug`, `info`, `warn`, `error`     | `debug`                    | ログ表示レベル                                                  |

### 非インタラクティブ(Config を変更)な設定値

| Param                  | Values                | Default      | Misc.                                                                      |
| ---------------------- | --------------------- | ------------ | -------------------------------------------------------------------------- |
| MAX_PLANE_ITERATION    | Positive Int          | 20           |                                                                            |
| PLANE_POINTS_THRESHOLD | Positive Int          | 500          | 検知する平面が含む point 数の下限                                          |
| MAX_ITERATION          | Positive Int          | 1000         | フィッティング実行時の反復回数                                             |
| INLIER_THRESHOLD       | Positive Float        | 1.0          | 平面・曲面方程式が算出された際に、その面からどれだけの距離の点を抽出するか |
| CAPTURE_ZOOM           | Positive Float        | 0.2          | キャプチャ画像生成時の拡大率, 決め打ち                                     |
| CAM_FRONT              | [float, float, float] | [ -1, -1, 1] | 焦点から見たカメラ方向                                                     |

## Development

### Test gui

`tests/test_gui.py`で対象フォルダパスを指定後、

```sh
poetry run pytest test/test_gui.py
```

### シミュレーションモデル作成

座標`(0, 0, 0)`から正方向にモデルを作成する。
これは、撮影カメラで 45 度角から撮影したくて、焦点(0, 0, 0), カメラ方向(-1, -1, 1)から撮るため。

FreeCAD で作成したモデルを点群化は、`stl2ply.py`を利用。

## FIXME: 課題

### キャプチャ時の cam_front 値

カメラのキャプチャのカメラ方向は決め打ちなので、自動化したい

## License

当プロジェクトは GPL ライセンス準拠です。
