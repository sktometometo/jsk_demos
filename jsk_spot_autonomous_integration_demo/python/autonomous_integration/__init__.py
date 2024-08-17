from typing import Any, Callable, Dict, List, Optional, Tuple

import numpy as np
import openai
import yaml

ARGUMENT_NAMES_AND_TYPES = Dict[str, str]
ARGUMENT_LIST = Dict[str, Any]
POSITION = np.ndarray
POSITION_SIMILARITY_FUNCTION = Callable[[POSITION, POSITION], float]
