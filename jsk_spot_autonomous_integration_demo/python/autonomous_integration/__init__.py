from typing import Any, Callable, Dict, List, Optional, Tuple

import numpy as np
from scipy.spatial.transform import Rotation as R

ARGUMENT_NAMES_AND_TYPES = List[Tuple[str, str]]
RESPONSE_NAMES_AND_TYPES = List[Tuple[str, str]]
ARGUMENT_LIST = Dict[str, Any]
RESPONSE_LIST = Dict[str, Any]
POSE_SIMILARITY_FUNCTION = Callable[[np.ndarray, R, np.ndarray, R], float]


def names_and_types_from_dict(data: List[Dict[str, str]]) -> List[Tuple[str, str]]:
    return [(d["name"], d["type"]) for d in data]


def names_and_types_to_dict(data: List[Tuple[str, str]]) -> List[Dict[str, str]]:
    return [{"name": d[0], "type": d[1]} for d in data]
