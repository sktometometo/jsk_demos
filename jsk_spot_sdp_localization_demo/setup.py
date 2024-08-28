from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=[
        "spot_localization_demo",
        "spot_demo",
    ],
    package_dir={
        "": "python",
    },
)

setup(**d)
