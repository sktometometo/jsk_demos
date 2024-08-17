from typing import Any, Callable, Dict, List, Optional, Tuple

import numpy as np
import openai
import yaml

from . import ARGUMENT_NAMES_AND_TYPES, POSITION, POSITION_SIMILARITY_FUNCTION


def cosine_similarity(vec1, vec2) -> float:
    return np.dot(vec1, vec2) / (np.linalg.norm(vec1) * np.linalg.norm(vec2))


class AutonomousIntegrator:

    def __init__(self, api_key: str):
        openai.api_key = api_key
        self.api_client = openai.OpenAI()

    def _get_embedding(self, text: str, model: str) -> np.ndarray:
        """
        Get the embedding of the given text.
        """
        response = self.api_client.embeddings.create(
            input=[text],
            model=model,
        )
        embeddings = response.data[0].embedding
        return np.array(embeddings)

    def _calc_semantic_similarity(
        self,
        description_api: str,
        description_intension: str,
    ) -> float:
        """
        Calculate the semantic distance between two descriptions.
        """
        return cosine_similarity(
            self._get_embedding(description_api, "text-embedding-3-small"),
            self._get_embedding(description_intension, "text-embedding-3-small"),
        )

    def select_api(
        self,
        description_intension: str,
        position_intension: POSITION,
        list_api: List[
            Tuple[str, ARGUMENT_NAMES_AND_TYPES, POSITION, POSITION_SIMILARITY_FUNCTION]
        ],
        threshold: float = 0.5,
        threshold_pos: float = 0.5,
    ) -> Optional[
        Tuple[str, ARGUMENT_NAMES_AND_TYPES, POSITION, POSITION_SIMILARITY_FUNCTION]
    ]:
        """
        Select the most suitable API for the given intension and position.
        """
        max_similarity = threshold
        max_similarity_position = threshold_pos
        selected_api = None
        for (
            description_api,
            api_arguments,
            api_position,
            func_position_similiarity,
        ) in list_api:
            similarity = self._calc_semantic_similarity(
                description_intension, description_api
            )
            similarity_position = func_position_similiarity(
                position_intension, api_position
            )
            if (
                similarity > max_similarity
                and similarity_position > max_similarity_position
            ):
                max_similarity = similarity
                max_similarity_position = similarity_position
                selected_api = (
                    description_api,
                    api_arguments,
                    api_position,
                    func_position_similiarity,
                )
        return selected_api
