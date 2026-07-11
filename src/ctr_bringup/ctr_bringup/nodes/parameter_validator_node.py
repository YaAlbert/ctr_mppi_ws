from ctr_bringup.placeholder_node import create_placeholder_main


main = create_placeholder_main(
    package_name="ctr_bringup",
    node_name="parameter_validator_node",
    required_sections=("robot", "model", "mppi", "simulation", "safety", "tactile", "hardware"),
    note="Validates project YAML files and exits only when the ROS2 process is stopped.",
)


if __name__ == "__main__":
    main()
