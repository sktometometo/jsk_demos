from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import openai
import yaml

from . import (ARGUMENT_LIST, ARGUMENT_NAMES_AND_TYPES, POSITION,
               POSITION_SIMILARITY_FUNCTION)


class ArgumentCompletion:

    def __init__(self, api_key: str):
        openai.api_key = api_key
        self.api_client = openai.OpenAI()

    def _generate_argument_for_api(
        self,
        target_api_argument_name: str,
        target_api_argument_type: str,
        other_api_arguments: ARGUMENT_LIST,
        description_api: str,
        description_intension: str,
        arguments_intension: ARGUMENT_LIST,
    ) -> Any:
        """
        Generate an argument for the target API based on the given API description with openai API
        """
        prompt = """Generate an argument for the target API based on the given API description.    

### Example
Description of the intension: "Control the air conditionar"
        
Input arguments for the intension:
    Temperature: 27.0
    Mode: Cooling
        
Description of the API: "Control the cooler" 

Other input arguments for the API:


Argument (name: "Temperature" and type: "int") : 27

### Actual
Description of the intension: "{}"
 
Input arguments for the intension:
{}
        
Description of the API: "{}"

Other input arguments for the API:
{}

Argument (name: "{}" and type: "{}") : """.format(
            description_intension,
            yaml.dump(arguments_intension),
            description_api,
            yaml.dump(other_api_arguments),
            target_api_argument_name,
            target_api_argument_type,
        )
        response = self.api_client.completions.create(
            model="text-davinci-003",
            prompt=prompt,
            max_tokens=100,
        )
        print(response)
        return response.choices[0].text

    def generate_arguments(
        self,
        api_description: str,
        target_description: str,
        api_argument_names: List[ARGUMENT_NAMES_AND_TYPES],  # Name and type
        target_arguments: ARGUMENT_LIST,
    ) -> ARGUMENT_LIST:
        """
        Generate arguments for the target API based on the given API description.
        """
        arguments: Dict[str, Any] = {}
        for i, (target_api_argument_name, target_api_argument_type) in enumerate(
            api_argument_names
        ):
            arguments[target_api_argument_name] = self._generate_argument_for_api(
                target_api_argument_name,
                target_api_argument_type,
                arguments,
                api_description,
                target_description,
                target_arguments,
            )
        return arguments
