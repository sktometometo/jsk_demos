from typing import Any, Callable, Dict, List, Optional, Tuple

import numpy as np

ARGUMENT_NAMES_AND_TYPES = List[Tuple[str, str]]
RESPONSE_NAMES_AND_TYPES = List[Tuple[str, str]]
ARGUMENT_LIST = Dict[str, Any]
RESPONSE_LIST = Dict[str, Any]
POSITION = np.ndarray
POSITION_SIMILARITY_FUNCTION = Callable[[POSITION, POSITION], float]
