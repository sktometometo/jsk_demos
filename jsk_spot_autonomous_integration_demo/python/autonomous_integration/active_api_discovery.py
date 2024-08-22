from typing import Any, Callable, Dict, List, Optional, Tuple

import numpy as np
import openai

from . import (
    ARGUMENT_NAMES_AND_TYPES,
    RESPONSE_NAMES_AND_TYPES,
)


def cosine_similarity(vec1, vec2) -> float:
    return np.dot(vec1, vec2) / (np.linalg.norm(vec1) * np.linalg.norm(vec2))


class ActiveAPIDiscovery:

    def __init__(self, api_key: str):
        openai.api_key = api_key
        self.api_client = openai.OpenAI(api_key=api_key)
        print(f"openai.api_key: {openai.api_key}")

    def _get_embedding(
        self,
        description: str,
        arguments: ARGUMENT_NAMES_AND_TYPES,
        responses: RESPONSE_NAMES_AND_TYPES,
        model: str,
    ) -> np.ndarray:
        """
        Get the embedding of the given text.
        """
        text = "API description: " + description + "\n"
        text += "Arguments: " + str(arguments) + "\n"
        text += "Responses: " + str(responses)
        res = self.api_client.embeddings.create(
            input=[text],
            model=model,
        )
        embeddings = res.data[0].embedding
        return np.array(embeddings)

    def _calc_semantic_similarity(
        self,
        description_intension: str,
        arguments_intension: ARGUMENT_NAMES_AND_TYPES,
        response_intension: RESPONSE_NAMES_AND_TYPES,
        description_api: str,
        arguments_api: ARGUMENT_NAMES_AND_TYPES,
        response_api: RESPONSE_NAMES_AND_TYPES,
    ) -> float:
        """
        Calculate the semantic distance between two descriptions.
        """
        return cosine_similarity(
            self._get_embedding(
                description_api,
                arguments_api,
                response_api,
                "text-embedding-3-small",
            ),
            self._get_embedding(
                description_intension,
                arguments_intension,
                response_intension,
                "text-embedding-3-small",
            ),
        )

    def select_api(
        self,
        description_intension: str,
        description_argument_names_and_types: ARGUMENT_NAMES_AND_TYPES,
        description_response_names_and_types: RESPONSE_NAMES_AND_TYPES,
        list_api: List[Tuple[str, ARGUMENT_NAMES_AND_TYPES, RESPONSE_NAMES_AND_TYPES]],
        threshold: float = 0.5,
    ) -> Optional[Tuple[str, ARGUMENT_NAMES_AND_TYPES, RESPONSE_NAMES_AND_TYPES]]:
        """
        Select the most suitable API for the given intension and position.
        """
        max_similarity = threshold
        selected_api = None
        for (
            description_api,
            api_arguments,
            api_response,
        ) in list_api:
            similarity = self._calc_semantic_similarity(
                description_intension,
                description_argument_names_and_types,
                description_response_names_and_types,
                description_api,
                api_arguments,
                api_response,
            )
            print(f"{description_api}: similarity: {similarity}")
            if similarity > max_similarity:
                max_similarity = similarity
                selected_api = (
                    description_api,
                    api_arguments,
                    api_response,
                )
        return selected_api
