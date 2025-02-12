import json
import os
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, Tuple

import numpy as np
from autonomous_integration import (
    ARGUMENT_NAMES_AND_TYPES,
    RESPONSE_NAMES_AND_TYPES,
    names_and_types_from_dict,
    names_and_types_to_dict,
)
from autonomous_integration.active_api_discovery import ActiveAPIDiscovery
from autonomous_integration.autonomous_argument_completion import ArgumentCompletion
from autonomous_integration.sdp_utils import *


def load_params(filepath: str):
    with open(filepath, "r") as f:
        data = json.load(f)

    functions = [Function.from_dict(d) for d in data["functions"]]
    conditions = [
        (
            d["robot_position"],
            d["robot_direction"],
            d["intension"],
            d["id"],
        )
        for d in data["conditions"]
    ]
    return functions, conditions


@dataclass
class Function:
    name: str
    description: str
    argument_names_and_types: ARGUMENT_NAMES_AND_TYPES
    response_names_and_types: RESPONSE_NAMES_AND_TYPES
    position: Tuple[float, float, float]

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "Function":
        return cls(
            name=data["name"],
            description=data["description"],
            argument_names_and_types=names_and_types_from_dict(
                data["argument_names_and_types"]
            ),
            response_names_and_types=names_and_types_from_dict(
                data["response_names_and_types"]
            ),
            position=(data["position"][0], data["position"][1], data["position"][2]),
        )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "description": self.description,
            "argument_names_and_types": names_and_types_to_dict(
                self.argument_names_and_types
            ),
            "response_names_and_types": names_and_types_to_dict(
                self.response_names_and_types
            ),
            "position": [self.position[0], self.position[1], self.position[2]],
        }


class Environment:

    def __init__(
        self,
        robot_position: Tuple[float, float, float],
        robot_direction: float,
        functions: Dict[str, Function],
    ):
        self.robot_position = robot_position
        self.robot_direction = robot_direction
        self.functions = functions

    def get_api_list(
        self,
    ) -> List[
        Tuple[
            str,
            str,
            ARGUMENT_NAMES_AND_TYPES,
            RESPONSE_NAMES_AND_TYPES,
            Tuple[float, float, float],
        ]
    ]:
        return [
            (
                function.name,
                function.description,
                function.argument_names_and_types,
                function.response_names_and_types,
                function.position,
            )
            for function in self.functions.values()
        ]


def call_device(
    environment: Environment,
    intension: str,
    max_workers: int = 4,
) -> Optional[Tuple]:
    discovery = ActiveAPIDiscovery(max_workers=max_workers)
    completion = ArgumentCompletion()

    api_full_list = environment.get_api_list()
    api_short_list = [(api[1], api[2], api[3]) for api in api_full_list]
    print(f"api_short_list: {api_short_list}")
    similarity_list, target_api_list_short_with_similarity = discovery.select_api(
        intension,
        {},
        [],
        api_short_list,
    )
    target_api_list_short = [
        target_api_short
        for similarity, target_api_short in target_api_list_short_with_similarity
    ]
    target_api_list_full = [
        api_full_list[api_short_list.index(target_api_short)]
        for target_api_short in target_api_list_short
    ]
    #
    target_api_full = None
    distance_to_base = float("inf")
    for target_api_full_candidate, similarity in zip(
        target_api_list_full, similarity_list
    ):
        distance = np.linalg.norm(
            np.array(target_api_full_candidate[4])
            - np.array(environment.robot_position)
        )
        if distance < distance_to_base:
            target_api_full = target_api_full_candidate
            distance_to_base = distance
    if target_api_full is None:
        return None
    target_api_short = api_short_list[api_full_list.index(target_api_full)]
    target_api_args = completion.generate_arguments_for_api(
        intension,
        {},
        [],
        target_api_short[0],
        target_api_short[1],
        target_api_short[2],
    )
    # Call the dummy function
    return target_api_full, target_api_args
