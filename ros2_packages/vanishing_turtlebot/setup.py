from setuptools import setup
import os
from glob import glob

# import sys

package_name = "vanishing_turtlebot"

install_path = os.path.join("share", package_name)
models_files = []
for path, _, files in os.walk("models"):
    list_entry = (
        os.path.join(install_path, path),
        [os.path.join(path, f) for f in files if not f.startswith(".")],
    )
    models_files.append(list_entry)
# print(models_files, file=sys.stderr)

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*.xml")),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.rviz"),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.yaml"),
        ),
        (
            os.path.join("share", package_name, "config"),
            glob("config/*.perspective"),
        ),
        (
            os.path.join("share", package_name, "worlds"),
            glob("worlds/*.model"),
        ),
    ]
    + models_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="fix_jer",
    maintainer_email="jeremy.fix@centralesupelec.fr",
    description="A package for the labwork on turtlebot controled by the vanishing point",
    license="BSD",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "joy_teleop = vanishing_turtlebot.joy_teleop:main",
            "controller = vanishing_turtlebot.controller:main",
            "twist2pic = vanishing_turtlebot.twist2pic:main",
            "paramissue = vanishing_turtlebot.param_issue:main",
        ]
    },
)
