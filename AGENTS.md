# CTR MPPI Repository Instructions

## Source of truth

The current Ubuntu 22.04 repository and runtime environment are the
source of truth.

Some files may have been created previously on Windows. Treat those
files only as existing project assets. Do not assume that Windows build
results, paths, generated files, package states, or prior milestone
completion are valid in Ubuntu.

When documentation conflicts with the current repository, installed ROS2
environment, source code, build results, or tests:

1. report the conflict;
2. use the current Ubuntu environment as the operational source of truth;
3. preserve useful existing source code where possible;
4. do not silently delete or overwrite files;
5. record unresolved conflicts in docs/18_unresolved_items.md.

## Required reading

Before editing code, read:

- README.md
- CODEX_TASK.md
- CURRENT_STATUS.md
- all files under docs/
- all YAML files under config/
- existing source files
- existing tests

## Working rules

- Work on one milestone at a time.
- The first task is an audit only; do not modify files unless explicitly asked.
- Do not assume that milestones completed on Windows are valid.
- Verify each claimed milestone using files, build results, and tests.
- Use ROS2 Humble and Ubuntu 22.04 as the target environment.
- Do not hard-code robot, MPPI, tactile, safety, or hardware parameters.
- Do not invent missing hardware values.
- Preserve simulation/hardware interface separation.
- Do not send raw MPPI commands directly to physical hardware.
- Never delete existing code merely because it differs from documentation.
- Ask before destructive operations.
- Do not use sudo unless explicitly approved.
- Do not install packages unless the missing dependency has been identified and reported.

## Required report format

After every task, report:

1. Summary
2. Files inspected
3. Files created
4. Files modified
5. Commands run
6. Build results
7. Test results
8. Current repository conflicts
9. Remaining TODO IDs
10. Missing parameters
11. Known limitations
12. Recommended next task
