cmake_minimum_required(VERSION 3.0.2)
project(jsk_spot_autonomous_integration_demo)

find_package(catkin REQUIRED COMPONENTS catkin_virtualenv)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS catkin_virtualenv
)

catkin_generate_virtualenv(
    INPUT_REQUIREMENTS requirements.txt
    PYTHON_INTERPRETER python3.8
    USE_SYSTEM_PACKAGES TRUE
    CHECK_VENV FALSE
)

catkin_install_python(
  PROGRAMS node_scripts/example_active_api_discovery.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
catkin_install_python(
  PROGRAMS node_scripts/example_autonomous_argument_completion.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)