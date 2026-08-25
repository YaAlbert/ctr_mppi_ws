from setuptools import find_packages, setup

package_name = "ctr_sim"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="TODO-OWNER-001",
    maintainer_email="todo@example.com",
    description="CTR simulation loop and manual command publisher.",
    license="TODO-LICENSE-001",
    entry_points={
        "console_scripts": [
            "simulator_node = ctr_sim.nodes.simulator_node:main",
            "manual_command_publisher = ctr_sim.nodes.manual_command_publisher:main",
            "development_target_selector_node = ctr_sim.nodes.development_target_selector_node:main",
        ]
    },
)
