from concurrent.futures import ThreadPoolExecutor
from typing import Any, Callable, Dict, List, Optional, Tuple

import numpy as np
import rospy
from openai_ros.srv import Embedding, EmbeddingRequest

from . import ARGUMENT_NAMES_AND_TYPES, RESPONSE_NAMES_AND_TYPES


def cosine_similarity(vec1, vec2) -> float:
    return float(np.dot(vec1, vec2) / (np.linalg.norm(vec1) * np.linalg.norm(vec2)))


class ActiveAPIDiscovery:

    def __init__(
        self, service_name: str = "/openai/get_embedding", max_workers: int = 5
    ):
        self._max_workers = max_workers
        rospy.wait_for_service(service_name, timeout=5.0)
        self.get_embedding = rospy.ServiceProxy(service_name, Embedding)

    def _get_embedding(
        self,
        description: str,
        arguments: ARGUMENT_NAMES_AND_TYPES,
        responses: RESPONSE_NAMES_AND_TYPES,
    ) -> np.ndarray:
        """
        Get the embedding of the given text.
        """
        text = "API description: " + description + "\n"
        text += "Arguments: " + str(arguments) + "\n"
        text += "Responses: " + str(responses)
        # rospy.loginfo(f"Requesting embedding for: {text}")
        res = self.get_embedding(EmbeddingRequest(prompt=text))
        embeddings = res.embedding
        # rospy.loginfo(f"Received embedding for {text}")
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
            ),
            self._get_embedding(
                description_intension,
                arguments_intension,
                response_intension,
            ),
        )

    def select_api(
        self,
        description_intension: str,
        argument_names_and_types_intension: ARGUMENT_NAMES_AND_TYPES,
        response_names_and_types_intension: RESPONSE_NAMES_AND_TYPES,
        list_api: List[Tuple[str, ARGUMENT_NAMES_AND_TYPES, RESPONSE_NAMES_AND_TYPES]],
        threshold: float = 0.5,
    ) -> List[
        Tuple[
            List[float],
            List[
                Tuple[
                    float,
                    Tuple[str, ARGUMENT_NAMES_AND_TYPES, RESPONSE_NAMES_AND_TYPES],
                ]
            ],
        ]
    ]:
        """
        Select the most suitable API for the given intension and position.
        """

        def compute_similarity(api_item):
            description_api, api_arguments, api_response = api_item
            similarity = self._calc_semantic_similarity(
                description_intension,
                argument_names_and_types_intension,
                response_names_and_types_intension,
                description_api,
                api_arguments,
                api_response,
            )
            print(f"{description_api}: similarity: {similarity}")
            if similarity > threshold:
                selected_api = (
                    description_api,
                    api_arguments,
                    api_response,
                )
                return (similarity, selected_api)
            else:
                return (similarity, None)

        with ThreadPoolExecutor(max_workers=self._max_workers) as executor:
            results = executor.map(compute_similarity, list_api)
            selected_apis = [result for result in results if result[1] is not None]
            selected_apis = sorted(selected_apis, key=lambda x: -x[0])
            similarity_list = [result[0] for result in selected_apis]
            return similarity_list, selected_apis
