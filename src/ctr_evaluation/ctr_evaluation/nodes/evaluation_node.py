from ctr_bringup.placeholder_node import create_placeholder_main


main = create_placeholder_main(
    package_name="ctr_evaluation",
    node_name="evaluation_node",
    required_sections=("robot", "model", "simulation"),
    note="TODO-DATA-001: experiment metrics and data registration are not implemented in Milestone 1.",
)


if __name__ == "__main__":
    main()
