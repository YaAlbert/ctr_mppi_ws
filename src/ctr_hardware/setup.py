from setuptools import find_packages, setup

package_name = "ctr_hardware"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="TODO-OWNER-001",
    maintainer_email="todo@example.com",
    description="Milestone 1 placeholder package for mock and physical hardware interfaces.",
    license="TODO-LICENSE-001",
    entry_points={
        "console_scripts": [
            "mock_hardware_node = ctr_hardware.nodes.mock_hardware_node:main",
            "physical_hardware_node = ctr_hardware.nodes.physical_hardware_node:main",
        ]
    },
)
