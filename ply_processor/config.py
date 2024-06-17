class Config(dict):
    MAX_PLANE_ITERATION = 20
    PLANE_POINTS_THRESHOLD = 5000
    MAX_ITERATION = 1000
    LOOP = 10
    FILEPATH = "data/stained/segmented.ply"
    INLIER_THRESHOLD = 1.0
    MODE = "prod"
    MODEL = {"h_bottom": 10.0, "h_top": 30.0, "r": 35.0 / 2}
