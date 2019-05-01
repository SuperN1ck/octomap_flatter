from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=["octomap_flatter"],
    scripts=["scripts/boundary_detector"],
    package_dir={"": "src"}
)

setup(**d)
