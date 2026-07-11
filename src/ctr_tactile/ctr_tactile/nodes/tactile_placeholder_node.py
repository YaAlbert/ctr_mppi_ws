from ctr_bringup.placeholder_node import create_placeholder_main


main = create_placeholder_main(
    package_name="ctr_tactile",
    node_name="tactile_placeholder_node",
    required_sections=("tactile", "safety"),
    note="TODO-SNS-001: tactile hardware processing is not implemented in Milestone 1.",
)


if __name__ == "__main__":
    main()
