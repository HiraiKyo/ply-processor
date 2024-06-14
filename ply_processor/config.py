class Config(dict):
    MAX_ITERATION = 1000
    LOOP = 10
    FILEPATH = "data/stained/segmented.ply"
    INLIER_THRESHOLD = 1.0
    MODE = "dev"
    MODEL = {"h_bottom": 10.0, "h_top": 30.0, "r": 35.0 / 2}
