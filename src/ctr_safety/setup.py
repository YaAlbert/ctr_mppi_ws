from setuptools import find_packages, setup

package_name = "ctr_safety"

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
    description="Milestone 1 placeholder package for CTR safety supervision.",
    license="TODO-LICENSE-001",
    entry_points={"console_scripts": ["safety_supervisor_node = ctr_safety.nodes.safety_supervisor_node:main"]},
)
