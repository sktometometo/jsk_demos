cmake_minimum_required(VERSION 3.0.2)
project(jsk_spot_autonomous_integration_demo)

find_package(catkin REQUIRED COMPONENTS catkin_virtualenv)

catkin_python_setup()

catkin_generate_virtualenv(
    INPUT_REQUIREMENTS requirements.txt
    PYTHON_INTERPRETER python3.8
    USE_SYSTEM_PACKAGES FALSE
    CHECK_VENV FALSE
)

catkin_package()
