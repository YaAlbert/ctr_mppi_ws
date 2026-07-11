from ctr_bringup.placeholder_node import create_placeholder_main


main = create_placeholder_main(
    package_name="ctr_hardware",
    node_name="physical_hardware_node",
    required_sections=("robot", "hardware", "safety"),
    note="TODO-HW-003: physical motor driver is intentionally not implemented in Milestone 1.",
)


if __name__ == "__main__":
    main()
