import time
from .singleton import Singleton
from .config import ConfigBase


class Color:
    BLUE = "\033[1;34m"
    BLUEL = "\033[0;34m"
    GREEN = "\033[1;32m"
    GREENL = "\033[0;32m"
    CYAN = "\033[1;36m"
    CYANL = "\033[0;36m"
    RED = "\033[1;31m"
    REDL = "\033[0;31m"
    PURPLE = "\033[1;35m"
    PURPLEL = "\033[0;35m"
    YELLOW = "\033[1;33m"
    BROWN = "\033[0;33m"
    WHITE = "\033[1;37m"
    GRAYL = "\033[0;37m"
    GRAYD = "\033[1;30m"
    BLACK = "\033[0;30m"
    ENDC = "\033[0m"


class Logger(Singleton):
    def init(self, config: ConfigBase):
        self.config = config
        self.log_filename = f"log_{time.strftime('%Y%m%d_%H%M%S')}.log"
        self.logs = []

    def debug(self, message):
        if self.config.LOG_LEVEL == "debug":
            self.logs.append(f"[DEBUG] {message}")
            print(f"{Color.GRAYD}[DEBUG]{Color.ENDC} {message}")

    def info(self, message):
        if self.config.LOG_LEVEL in ["info", "debug"]:
            self.logs.append(f"[INFO] {message}")
            print(f"{Color.CYAN}[INFO]{Color.ENDC} {message}")

    def warn(self, message):
        if self.config.LOG_LEVEL in ["info", "debug", "warn"]:
            self.logs.append(f"[WARN] {message}")
            print(f"{Color.YELLOW}[WARN]{Color.ENDC} {message}")

    def error(self, message, code=None):
        if self.config.LOG_LEVEL in ["info", "debug", "error", "warn"]:
            self.logs.append(f"[ERROR] {message}, code: {code}")
            print(f"{Color.RED}[ERROR]{Color.ENDC} {message}")

    def export(self, dir_name):
        with open(f"{dir_name}/{self.log_filename}", "w") as f:
            for log in self.logs:
                f.write(log + "\n")
        print(f"Log has been exported to {dir_name}/{self.log_filename}")
