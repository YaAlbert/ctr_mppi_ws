from ctr_bringup.placeholder_node import create_placeholder_main


main = create_placeholder_main(
    package_name="ctr_hardware",
    node_name="mock_hardware_node",
    required_sections=("robot", "hardware", "safety"),
    note="TODO-HW-001: mock hardware feedback is not implemented in Milestone 1.",
)


if __name__ == "__main__":
    main()
