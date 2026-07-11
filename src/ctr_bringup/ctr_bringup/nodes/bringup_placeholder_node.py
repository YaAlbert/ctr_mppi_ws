from ctr_bringup.placeholder_node import create_placeholder_main


main = create_placeholder_main(
    package_name="ctr_bringup",
    node_name="bringup_placeholder_node",
    required_sections=("robot", "simulation", "hardware", "safety"),
)


if __name__ == "__main__":
    main()
