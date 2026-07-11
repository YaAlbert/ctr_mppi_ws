from ctr_bringup.placeholder_node import create_placeholder_main


main = create_placeholder_main(
    package_name="ctr_model",
    node_name="model_placeholder_node",
    required_sections=("robot", "model"),
    note="TODO-MODEL-004: Python forward kinematics is not implemented in Milestone 1.",
)


if __name__ == "__main__":
    main()
