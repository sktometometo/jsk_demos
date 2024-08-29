from typing import Any, Callable, Dict, List, Optional, Tuple

import numpy as np
from scipy.spatial.transform import Rotation as R

ARGUMENT_NAMES_AND_TYPES = List[Tuple[str, str]]
RESPONSE_NAMES_AND_TYPES = List[Tuple[str, str]]
ARGUMENT_LIST = Dict[str, Any]
RESPONSE_LIST = Dict[str, Any]
POSE_SIMILARITY_FUNCTION = Callable[[np.ndarray, R, np.ndarray, R], float]
