from typing import Any, Dict
from functools import lru_cache

import yaml

try:
    from yaml import CLoader as Loader
except ImportError:
    from yaml import Loader



@lru_cache(maxsize=None)
def load_reference_file(filepath: str) -> Dict[str, Any]:
    """
    Loads the file with reference pose. Caches every requested file.

    :param filepath: Path to the file.
    :type filepath: str
    :return: Dictionary containing pose structure or transforms.
    :rtype: Dict[str, Any]
    """
    with open(filepath, 'r') as f:
        return yaml.load(f, Loader=Loader)
