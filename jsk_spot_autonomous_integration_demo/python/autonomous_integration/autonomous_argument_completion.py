from typing import Any, Dict, List, Optional, Tuple

from click import argument
import openai
import yaml

from . import ARGUMENT_LIST, ARGUMENT_NAMES_AND_TYPES, RESPONSE_NAMES_AND_TYPES


def generate_prompt_example(
    target_api_argument_name: str,
    target_api_argument_type: str,
    description_intension: str,
    arguments_intension: ARGUMENT_LIST,
    response_intension: RESPONSE_NAMES_AND_TYPES,
    description_api: str,
    rest_args_api: ARGUMENT_LIST,
    response_api: RESPONSE_NAMES_AND_TYPES,
) -> str:
    """
    Generate a prompt example for the target API based on the given API description.
    """
    prompt = """Description of the intension: "{}"
        
Input arguments for the intension:
{}

Response names and types of the intension:
{}
        
Description of the API: "{}" 

Other input arguments for the API:
{}

Response names and types of the API:
{}

Argument for API (name: "{}" and type: "{}") : """.format(
        description_intension,
        arguments_intension,
        response_intension,
        description_api,
        rest_args_api,
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
        response_api: RESPONSE_NAMES_AND_TYPES,
        other_api_arguments: ARGUMENT_LIST,
        description_intension: str,
        arguments_intension: ARGUMENT_LIST,
        response_intension: RESPONSE_NAMES_AND_TYPES,
    ) -> Any:
        """
        Generate an argument for the target API based on the given API description with openai API
        """
        prompt = "Generate an argument for the target API based on the given API description.\n"
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
                {},
                [],
            )
            + "27\n"
        )
        prompt += "### Actual\n"
        prompt += generate_prompt_example(
            target_api_argument_name,
            target_api_argument_type,
            description_intension,
            arguments_intension,
            response_intension,
            description_api,
            other_api_arguments,
            [],
        )
        response = self.api_client.completions.create(
            model="gpt-3.5-turbo-instruct",
            prompt=prompt,
            max_tokens=100,
            stop=["\n"],
        )
        print(response)
        return response.choices[0].text

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
                response_api,
                arguments,
                description_intension,
                arguments_intension,
                response_intension,
            )
        return arguments
