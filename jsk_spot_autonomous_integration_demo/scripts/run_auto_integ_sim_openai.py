#!/usr/bin/env python

import argparse
import os
from typing import Any, Callable, Dict, List, Optional, Tuple

import rospkg
import rospy
from autonomous_integration_simulation.env import *
from autonomous_integration_simulation.openai_tool_calling import call_device_openai


def main(param_file: Optional[str] = None, openai_key: str = ""):
    rospy.init_node("demo")

    package_path = rospkg.RosPack().get_path("jsk_spot_autonomous_integration_demo")
    functions, conditions = load_params(
        os.path.join(package_path, "config", "demo.json")
        if param_file is None
        else param_file
    )

    print(functions)
    print(conditions)

    for condition in conditions:
        environment = Environment(
            robot_position=condition[0],
            robot_direction=condition[1],
            functions={f.name: f for f in functions},
        )
        print(environment)
        print(condition)
        result = call_device_openai(environment, condition[2], openai_key)
        # print(result)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--param_file", type=str)
    parser.add_argument("--openai_key", type=str)
    args = parser.parse_args()

    main(args.param_file, args.openai_key)
