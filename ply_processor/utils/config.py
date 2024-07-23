from .singleton import Singleton


class ConfigBase(dict, Singleton):
    LOG_LEVEL: str = "debug"
    MODE: str = "dev"
