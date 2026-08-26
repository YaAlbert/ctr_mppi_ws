import os
from glob import glob

from setuptools import find_packages, setup

package_name = "ctr_bringup"

config_files = glob(os.path.join("..", "..", "config", "*.yaml"))
config_files.extend(glob(os.path.join("..", "..", "config", "*.rviz")))

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", config_files),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="TODO-OWNER-001",
    maintainer_email="todo@example.com",
    description="Milestone 1 launch and parameter validation scaffolding.",
    license="TODO-LICENSE-001",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "parameter_validator_node = ctr_bringup.nodes.parameter_validator_node:main",
            "bringup_placeholder_node = ctr_bringup.nodes.bringup_placeholder_node:main",
            "ctr-runtime-candidate-validate-only = ctr_bringup.runtime_candidate_validate_only:main",
        ],
    },
)
