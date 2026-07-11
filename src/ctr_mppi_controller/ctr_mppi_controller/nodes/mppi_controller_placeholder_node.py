from ctr_bringup.placeholder_node import create_placeholder_main


main = create_placeholder_main(
    package_name="ctr_mppi_controller",
    node_name="mppi_controller_placeholder_node",
    required_sections=("robot", "model", "mppi", "safety"),
    note="TODO-MPPI-001: MPPI optimization is intentionally not implemented in Milestone 1.",
)


if __name__ == "__main__":
    main()
