import os
from enum import Enum
os.system("")  # enables ansi escape characters in terminal


class TERM_CONTROLS(Enum):
    ENDC = "\033[0m"
    BOLD = "\033[1m"
    UNDERLINE = "\033[4m"

    def __str__(self):
        return self.value


class TERM_COLORS(Enum):
    BLACK = "\033[90m"
    RED = "\033[91m"
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    BLUE = "\033[94m"
    MAGENTA = "\033[95m"
    CYAN = "\033[96m"
    WHITE = "\033[97m"

    def __str__(self):
        return self.value
