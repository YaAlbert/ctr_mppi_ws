from setuptools import find_packages, setup

package_name = "ctr_evaluation"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy", "PyYAML", "matplotlib"],
    zip_safe=True,
    maintainer="TODO-OWNER-001",
    maintainer_email="todo@example.com",
    description="Milestone 1 placeholder package for CTR metrics and evaluation.",
    license="TODO-LICENSE-001",
    entry_points={
        "console_scripts": [
            "evaluation_node = ctr_evaluation.nodes.evaluation_node:main",
            "compare_results = ctr_evaluation.compare_results:main",
            "ctr_run_evaluation = ctr_evaluation.run_evaluation:main",
        ]
    },
)
