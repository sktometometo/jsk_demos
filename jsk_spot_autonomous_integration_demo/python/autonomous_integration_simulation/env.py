import yaml
import rospkg
import os

from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, Tuple
from autonomous_integration import (
    ARGUMENT_NAMES_AND_TYPES,
    RESPONSE_NAMES_AND_TYPES,
    names_and_types_from_dict,
    names_and_types_to_dict,
)
from autonomous_integration.sdp_utils import *
from matplotlib import pyplot as plt, scale


@dataclass
class Function:
    name: str
    description: str
    argument_names_and_types: ARGUMENT_NAMES_AND_TYPES
    response_names_and_types: RESPONSE_NAMES_AND_TYPES
    position: Tuple[float, float]

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
            position=(data["position"][0], data["position"][1]),
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
            "position": [self.position[0], self.position[1]],
        }


class Environment:

    def __init__(
        self,
        filename: Optional[str] = None,
    ):
        self.functions: Dict[str, Function] = {}
        self.robot_position: Tuple[float, float] = (0.0, 0.0)
        self.robot_direction: float = 0.0
        if filename is not None:
            self.load(filename)

    def load(self, filename: str):
        with open(filename, "r") as file:
            data = yaml.safe_load(file)
        self.functions = {
            function["name"]: Function.from_dict(function)
            for function in data["functions"]
        }
        self.robot_position = (
            data["robot"]["position"][0],
            data["robot"]["position"][1],
        )
        self.robot_direction = data["robot"]["direction"]

    def show_environment(self):
        fig, ax = plt.subplots()
        # Plot functions
        for function in self.functions.values():
            ax.text(
                function.position[0],
                function.position[1],
                function.description,
                fontsize=12,
            )
        ax.scatter(
            [function.position[0] for function in self.functions.values()],
            [function.position[1] for function in self.functions.values()],
            marker="s",d
        )
        # Plot robot
        ax.text(
            self.robot_position[0],
            self.robot_position[1],
            "Robot",
            fontsize=12,
        )
        ax.quiver(
            self.robot_position[0],
            self.robot_position[1],
            self.robot_direction,
            scale=10,
        )
        # ax.set_xlim(-10, 10)
        # ax.set_ylim(-10, 10)
        plt.show()

    def get_api_list(
        self,
    ) -> List[
        Tuple[
            str,
            ARGUMENT_NAMES_AND_TYPES,
            RESPONSE_NAMES_AND_TYPES,
            Tuple[float, float],
        ]
    ]:
        return [
            (
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
):
    api_full_list = environment.get_api_list()


def main():
    package_path = rospkg.RosPack().get_path("jsk_spot_autonomous_integration_demo")

    environment = Environment(os.path.join(package_path, "config", "demo.yaml"))
    environment.show_environment()
