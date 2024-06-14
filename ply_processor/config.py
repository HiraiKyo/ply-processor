class Config(dict):
    MAX_ITERATION = 1000
    LOOP = 10
    FILEPATH = "data/stained/segmented.ply"
    INLIER_THRESHOLD = 0.1
    MODE = "dev"
    MODEL = {"h_bottom": 7.0, "h_top": 30.0, "r": 35.0 / 2}
