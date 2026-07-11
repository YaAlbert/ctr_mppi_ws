from setuptools import find_packages, setup

package_name = "ctr_model"

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
    description="CTR model interfaces and approximate simulation scaffold.",
    license="TODO-LICENSE-001",
    entry_points={"console_scripts": ["model_placeholder_node = ctr_model.nodes.model_placeholder_node:main"]},
)
