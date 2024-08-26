from typing import Any, Dict, List, Optional, Tuple

import openai
import yaml

from . import ARGUMENT_LIST, ARGUMENT_NAMES_AND_TYPES, RESPONSE_NAMES_AND_TYPES


def str_to_bool(s: str) -> bool:
    if s.replace(" ", "").lower() in ["true", "1", "t", "y", "yes"]:
        return True
    elif s.replace(" ", "").lower() in ["false", "0", "f", "n", "no"]:
        return False
    else:
        raise ValueError(f'Input cannot be converted to bool.: "{s}"')


def generate_prompt_example(
    target_api_argument_name: str,
    target_api_argument_type: str,
    description_intension: str,
    arguments_intension: ARGUMENT_LIST,
    response_intension: RESPONSE_NAMES_AND_TYPES,
    description_api: str,
    arguments_api: ARGUMENT_NAMES_AND_TYPES,
    response_api: RESPONSE_NAMES_AND_TYPES,
) -> str:
    """
    Generate a prompt example for the target API based on the given API description.
    """
    prompt = """=== Intension part ===
Description of the intension: "{}"

Input arguments for the intension:
{}

Response names and types of the intension:
{}

=== API part ===
Description of the API: "{}"

Input argument names and types for the API:
{}

Response names and types of the API:
{}

Argument for API (name: "{}" and type: "{}") :""".format(
        description_intension,
        arguments_intension,
        response_intension,
        description_api,
        arguments_api,
        response_api,
        target_api_argument_name,
        target_api_argument_type,
    )
    return prompt


class ArgumentCompletion:

    def __init__(self, api_key: str):
        openai.api_key = api_key
        self.api_client = openai.OpenAI(api_key=api_key)

    def _generate_argument_for_api(
        self,
        target_api_argument_name: str,
        target_api_argument_type: str,
        description_api: str,
        arguments_api: ARGUMENT_NAMES_AND_TYPES,
        response_api: RESPONSE_NAMES_AND_TYPES,
        description_intension: str,
        arguments_intension: ARGUMENT_LIST,
        response_intension: RESPONSE_NAMES_AND_TYPES,
    ) -> Any:
        """
        Generate an argument for the target API based on the given API description with openai API
        """
        prompt = "Generate arguments for the target API based on the given intension by following examples.\n"
        prompt += "\n"
        prompt += "### Example\n"
        prompt += (
            generate_prompt_example(
                "Temperature",
                "int",
                "Control the air conditionar",
                {"Temperature": 27.0, "Mode": "Cooling"},
                [],
                "Control the cooler",
                [],
                [],
            )
            + "27\n"
        )
        prompt += "\n"
        prompt += "### Example\n"
        prompt += (
            generate_prompt_example(
                "mode",
                "string",
                "Open the lock of the box",
                {},
                [],
                "Control the lock at the box in ABCD room",
                [],
                [],
            )
            + "unlock\n"
        )
        prompt += "\n"
        prompt += "### Example\n"
        prompt += (
            generate_prompt_example(
                "arg0",
                "string",
                "Open door lock",
                {},
                [],
                "SDP Lock 73B2: Key control",
                [],
                [],
            )
            + "unlock\n"
        )
        prompt += "\n"
        prompt += "### Actual\n"
        prompt += generate_prompt_example(
            target_api_argument_name,
            target_api_argument_type,
            description_intension,
            arguments_intension,
            response_intension,
            description_api,
            arguments_api,
            response_api,
        )
        response = self.api_client.completions.create(
            model="gpt-3.5-turbo-instruct",
            prompt=prompt,
            max_tokens=100,
            stop=["\n"],
        )
        response_text = response.choices[0].text
        print(f"prompt: {prompt}")
        print(f"response: {response}")
        if target_api_argument_type == "int":
            return int(response_text)
        elif target_api_argument_type == "float":
            return float(response_text)
        elif target_api_argument_type == "bool":
            return str_to_bool(response_text)
        elif target_api_argument_type == "string":
            return response_text

    def generate_arguments_for_api(
        self,
        description_intension: str,
        arguments_intension: ARGUMENT_LIST,
        response_intension: RESPONSE_NAMES_AND_TYPES,
        description_api: str,
        arguments_api: ARGUMENT_NAMES_AND_TYPES,
        response_api: RESPONSE_NAMES_AND_TYPES,
    ) -> ARGUMENT_LIST:
        """
        Generate arguments for the target API based on the given API description.
        """
        arguments: Dict[str, Any] = {}
        for i, (target_api_argument_name, target_api_argument_type) in enumerate(
            arguments_api,
        ):

            arguments[target_api_argument_name] = self._generate_argument_for_api(
                target_api_argument_name,
                target_api_argument_type,
                description_api,
                arguments_api,
                response_api,
                description_intension,
                arguments_intension,
                response_intension,
            )
        return arguments
