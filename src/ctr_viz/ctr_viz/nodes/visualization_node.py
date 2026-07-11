from ctr_bringup.placeholder_node import create_placeholder_main


main = create_placeholder_main(
    package_name="ctr_viz",
    node_name="visualization_node",
    required_sections=("robot", "simulation"),
    note="TODO-FRAME-002: RViz markers and TF publishing are not implemented in Milestone 1.",
)


if __name__ == "__main__":
    main()
