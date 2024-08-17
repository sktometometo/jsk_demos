#!/usr/bin/env python

import argparse

from autonomous_integration.autonomous_argument_completion import (
    AutonomousArgumentCompletion,
)
from autonomous_integration.autonomous_integrator import AutonomousIntegrator

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--api-key", type=str, required=True)
    args = parser.parse_args()

    ai = AutonomousIntegrator(api_key=args.api_key)

    example_api_list = [
        (
            "Get the thermostat status",
            {
                "thermostat_id": str,
                "temperature": float,
                "humidity": float,
            },
            