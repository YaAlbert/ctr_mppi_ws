from setuptools import find_packages, setup

package_name = "ctr_mppi_controller"

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
    description="ROS-independent MPPI core and ROS2 controller wrapper.",
    license="TODO-LICENSE-001",
    entry_points={
        "console_scripts": [
            "mppi_controller_placeholder_node = ctr_mppi_controller.nodes.mppi_controller_placeholder_node:main",
            "mppi_controller_node = ctr_mppi_controller.nodes.mppi_controller_node:main",
            "reference_manager_node = ctr_mppi_controller.nodes.reference_manager_node:main",
        ]
    },
)
