from ctr_bringup.placeholder_node import create_placeholder_main


main = create_placeholder_main(
    package_name="ctr_state_estimator",
    node_name="state_estimator_node",
    required_sections=("robot", "model", "tactile"),
    note="TODO-ROS-002: fused state publication is not implemented in Milestone 1.",
)


if __name__ == "__main__":
    main()
