class Config(dict):
    MAX_ITERATION = 1000
    LOOP = 10
    FILEPATH = "data/stained/segmented_2.ply"
    INLIER_THRESHOLD = 0.1
    MODE = "dev"
    MODEL = {
        "h": 50.0,
        "r": 35.0
    }
