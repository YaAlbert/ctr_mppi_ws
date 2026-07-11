from ctr_bringup.placeholder_node import create_placeholder_main


main = create_placeholder_main(
    package_name="ctr_safety",
    node_name="safety_supervisor_node",
    required_sections=("robot", "safety", "tactile", "hardware"),
    note="TODO-SAFE-010: safety state machine is not implemented in Milestone 1.",
)


if __name__ == "__main__":
    main()
