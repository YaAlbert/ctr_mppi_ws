"""Practical, explicitly selected Slice 7G simulator-only workflow."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import subprocess
import sys
from typing import Any

from ctr_bringup.parameter_validation import load_parameter_files, validate_or_raise
from ctr_bringup.slice_7g_profile import apply_slice_7g_development_simulation_profile
from ctr_model.approximate_model import ApproximateCTRModel
from ctr_mppi_controller.curved_lumen import CurvedLumen
from ctr_mppi_controller.lumen_factory import config_with_lumen_overrides, lumen_geometry_from_config
from ctr_sim.nodes.development_target_selector_node import (
    build_sampled_reachability_cloud,
    sampled_reachability_predicate,
    select_development_target,
)
from ctr_evaluation.metrics import sanitize_for_json
from ctr_evaluation.run_evaluation import (
    DEVELOPMENT_SIMULATION_DISCLAIMER,
    EvaluationOrchestrator,
    OrchestrationError,
    default_config_paths,
    parse_args as parse_evaluation_args,
    validate_development_output_root,
)


DEFAULT_SEEDS = (11, 22, 33)
DEFAULT_DURATION_SECONDS = 25.0
DEFAULT_SMOKE_DURATION_SECONDS = 5.0
TARGET_SOURCE_MODES = ("profile", "cli", "rviz")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run the explicit simulator-only Slice 7G development example."
    )
    parser.add_argument(
        "--development-simulation",
        action="store_true",
        help="Required opt-in; production authority and production attempts are never used.",
    )
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION_SECONDS)
    parser.add_argument("--smoke-duration", type=float, default=DEFAULT_SMOKE_DURATION_SECONDS)
    parser.add_argument("--seeds", nargs="+", type=int, default=list(DEFAULT_SEEDS))
    parser.add_argument("--output-root", default="")
    parser.add_argument("--skip-smoke", action="store_true")
    parser.add_argument(
        "--target-source",
        choices=TARGET_SOURCE_MODES,
        default="profile",
        help="Development target source: profile (default), cli, or interactive rviz.",
    )
    parser.add_argument("--target-x", type=float, default=None, help="CLI target or automated RViz candidate X, metres.")
    parser.add_argument("--target-y", type=float, default=None, help="CLI target or automated RViz candidate Y, metres.")
    parser.add_argument("--target-z", type=float, default=None, help="CLI target or automated RViz candidate Z, metres.")
    parser.add_argument(
        "--target-selection-timeout",
        type=float,
        default=0.0,
        help="Interactive RViz selection timeout in seconds; zero disables it.",
    )
    parser.add_argument(
        "--automated-rviz-evaluation",
        action="store_true",
        help=(
            "Publish the supplied target coordinates through the RViz PointStamped topic "
            "and run the complete evaluator instead of opening an interactive RViz session."
        ),
    )
    parser.add_argument(
        "--target-frame",
        choices=("world", "base_link"),
        default="base_link",
        help="Input frame for an automated RViz candidate; CLI targets are always base_link.",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        if args.development_simulation is not True:
            raise OrchestrationError("explicit --development-simulation opt-in is required")
        if not _positive_finite(args.duration) or not _positive_finite(args.smoke_duration):
            raise OrchestrationError("development durations must be positive and finite")
        if not args.seeds or any(type(seed) is not int or seed < 0 for seed in args.seeds):
            raise OrchestrationError("development seeds must be nonnegative integers")
        if len(set(args.seeds)) != len(args.seeds):
            raise OrchestrationError("development seeds must not contain duplicates")
        target = development_cli_target(args)
        if args.target_source == "rviz" and not args.automated_rviz_evaluation:
            if len(args.seeds) != 1:
                raise OrchestrationError("interactive RViz target selection requires exactly one seed")
            return run_interactive_rviz(args)
        root = _result_root(args.output_root)
        root.mkdir(mode=0o700, parents=True, exist_ok=False)
        print(f"WARNING: {DEVELOPMENT_SIMULATION_DISCLAIMER}", file=sys.stderr)
        print(f"Slice 7G development result root: {root}")

        attempts: list[dict[str, Any]] = []
        if not args.skip_smoke:
            smoke = run_one_pair(
                root=root,
                seed=11,
                duration=args.smoke_duration,
                smoke=True,
                target_source=args.target_source,
                target=target,
                target_frame=args.target_frame,
            )
            attempts.append(smoke)
            if smoke["status"] != "passed":
                paths = write_development_results(root, attempts)
                print(f"Slice 7G smoke test failed: {smoke['failure_reason']}", file=sys.stderr)
                print(json.dumps(paths, sort_keys=True))
                return 2

        for seed in args.seeds:
            attempts.append(
                run_one_pair(
                    root=root,
                    seed=seed,
                    duration=args.duration,
                    smoke=False,
                    target_source=args.target_source,
                    target=target,
                    target_frame=args.target_frame,
                )
            )

        paths = write_development_results(root, attempts)
        passed = all(item["status"] == "passed" for item in attempts)
        print(json.dumps(sanitize_for_json({"passed": passed, **paths}), indent=2, allow_nan=False))
        return 0 if passed else 3
    except Exception as exc:
        print(f"ctr_run_slice_7g_development failed: {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1


def run_one_pair(
    *,
    root: Path,
    seed: int,
    duration: float,
    smoke: bool,
    target_source: str = "profile",
    target: tuple[float, float, float] | None = None,
    target_frame: str = "base_link",
) -> dict[str, Any]:
    label = "smoke" if smoke else "example"
    group = f"slice_7g_development_{label}_seed_{seed}"
    result: dict[str, Any] | None = None
    try:
        argv = [
                "--development-simulation",
                "--experiment-group",
                group,
                "--task",
                "curved_lumen_navigation",
                "--curved-lumen-type",
                "circular_arc",
                "--scenario",
                "centerline_target",
                "--mppi-profile",
                "cylinder_fast",
                "--seed",
                str(seed),
                "--duration",
                format(duration, ".17g"),
                "--runtime-mode",
                "simulation",
                "--output-root",
                str(root),
            ]
        evaluation_target = target
        projection_distance = 0.0
        if target_source == "rviz":
            if target is None:
                raise OrchestrationError("automated RViz target source requires candidate coordinates")
            selection = resolve_automated_rviz_candidate(
                target,
                input_frame=target_frame,
                seed=seed,
            )
            if not selection.accepted or selection.validated_target is None:
                raise OrchestrationError(f"automated RViz candidate rejected: {selection.status}")
            evaluation_target = tuple(selection.validated_target)
            projection_distance = float(selection.projection_distance)
        if target_source in {"cli", "rviz"}:
            if evaluation_target is None:
                raise OrchestrationError(f"{target_source} target source requires target coordinates")
            argv.extend(["--target", *(format(value, ".17g") for value in evaluation_target)])
        argv.extend(["--development-target-source", target_source])
        if target_source == "rviz" and target is not None:
            argv.extend(
                [
                    "--development-raw-target",
                    *(format(value, ".17g") for value in target),
                    "--development-target-frame",
                    target_frame,
                    "--development-target-projection-distance",
                    format(projection_distance, ".17g"),
                ]
            )
        evaluation_args = parse_evaluation_args(argv)
        result = EvaluationOrchestrator(evaluation_args).run_pair()
        metrics = collect_metrics(result)
        failure_reason = validate_functional_result(
            result,
            metrics,
            require_navigation=not smoke,
        )
        orchestration = _read_json(Path(result["candidate_dir"]) / "orchestration.json")
        target_selection = target_selection_result(
            target_source=target_source,
            supplied_target=target,
            orchestration=orchestration,
            seed=seed,
        )
        return {
            "kind": label,
            "seed": seed,
            "requested_duration_seconds": duration,
            "status": "passed" if failure_reason is None else "failed",
            "failure_reason": failure_reason,
            "ros_domain_id": result["ros_domain_id"],
            "baseline_dir": result["baseline_dir"],
            "candidate_dir": result["candidate_dir"],
            "metrics": metrics,
            "target_selection": target_selection,
        }
    except Exception as exc:
        return {
            "kind": label,
            "seed": seed,
            "requested_duration_seconds": duration,
            "status": "failed",
            "failure_reason": f"{type(exc).__name__}: {exc}",
            "ros_domain_id": None if result is None else result.get("ros_domain_id"),
            "baseline_dir": None if result is None else result.get("baseline_dir"),
            "candidate_dir": None if result is None else result.get("candidate_dir"),
            "metrics": {},
            "target_selection": {
                "target_source": target_source,
                "raw_input_point": None if target is None else list(target),
                "raw_input_frame": target_frame,
                "validated_target": None,
                "controller_target_frame": "base_link",
                "projection_distance": 0.0,
                "acceptance_status": "failed",
                "orientation_used": False,
                "accepted_target_timestamp": None,
                "reference_pose_count": 0,
                "seed": seed,
                "result_status": "failed",
            },
        }


def development_cli_target(args: argparse.Namespace) -> tuple[float, float, float] | None:
    values = (args.target_x, args.target_y, args.target_z)
    supplied = tuple(value is not None for value in values)
    coordinates_required = args.target_source == "cli" or (
        args.target_source == "rviz" and args.automated_rviz_evaluation
    )
    if coordinates_required:
        if supplied != (True, True, True):
            raise OrchestrationError(
                f"target_source={args.target_source} requires --target-x, --target-y, and --target-z"
            )
        target = tuple(float(value) for value in values)
        if not all(math.isfinite(value) for value in target):
            raise OrchestrationError("target coordinates must be finite")
        if args.target_source == "cli" and args.target_frame != "base_link":
            raise OrchestrationError("CLI target coordinates must use target frame base_link")
        return target
    if any(supplied):
        raise OrchestrationError(
            "target coordinates require --target-source cli or automated RViz evaluation"
        )
    if args.automated_rviz_evaluation:
        raise OrchestrationError("--automated-rviz-evaluation requires --target-source rviz")
    if not _finite_number(args.target_selection_timeout) or args.target_selection_timeout < 0.0:
        raise OrchestrationError("target selection timeout must be finite and nonnegative")
    return None


def resolve_automated_rviz_candidate(
    raw_target: tuple[float, float, float],
    *,
    input_frame: str,
    seed: int,
):
    raw_config = load_parameter_files(default_config_paths([]))
    config = config_with_lumen_overrides(
        raw_config,
        enable_cylindrical_lumen=False,
        enable_curved_lumen=True,
        curved_lumen_type="circular_arc",
        cylinder_profile="cylinder_fast",
        random_seed=seed,
    )
    config = apply_slice_7g_development_simulation_profile(config, enabled=True)
    validate_or_raise(config)
    geometry = lumen_geometry_from_config(config)
    if not isinstance(geometry, CurvedLumen):
        raise OrchestrationError("automated RViz evaluation requires CurvedLumen geometry")
    reachability = sampled_reachability_predicate(
        build_sampled_reachability_cloud(ApproximateCTRModel(config), config),
        float(config["goal"]["tolerance"]),
    )
    selection_config = config["simulation"]["development_target_selection"]
    return select_development_target(
        raw_target,
        input_frame=input_frame,
        target_source="rviz",
        geometry=geometry,
        controller_frame=str(config["reference"]["frame_id"]),
        world_frame=str(config["robot"]["frames"]["world"]),
        projection_limit=float(selection_config["projection_limit"]),
        reachable=reachability,
        accepted_target_timestamp=0.0,
        seed=seed,
    )


def interactive_rviz_command(args: argparse.Namespace) -> list[str]:
    return [
        "ros2",
        "launch",
        "ctr_bringup",
        "slice_7g_development_visual.launch.py",
        "development_simulation:=true",
        "target_source:=rviz",
        "wait_for_target:=true",
        f"target_selection_timeout:={float(args.target_selection_timeout):.17g}",
        f"seed:={int(args.seeds[0])}",
    ]


def run_interactive_rviz(args: argparse.Namespace) -> int:
    command = interactive_rviz_command(args)
    print("Starting interactive RViz target selection:")
    print(" ".join(command))
    try:
        return int(subprocess.run(command, check=False).returncode)
    except OSError as exc:
        raise OrchestrationError(f"could not start interactive RViz launch: {exc}") from exc


def target_selection_result(
    *,
    target_source: str,
    supplied_target: tuple[float, float, float] | None,
    orchestration: dict[str, Any],
    seed: int,
) -> dict[str, Any]:
    selection = orchestration.get("development_target_selection", {})
    if not isinstance(selection, dict):
        selection = {}
    validated = selection.get("validated_target", orchestration.get("validated_target"))
    requested = selection.get("raw_input_point", orchestration.get("requested_target"))
    accepted = (
        isinstance(validated, list)
        and len(validated) == 3
        and all(_finite_number(value) for value in validated)
    )
    return {
        "target_source": target_source,
        "raw_input_point": list(supplied_target) if supplied_target is not None else requested,
        "raw_input_frame": selection.get("raw_input_frame", "base_link"),
        "validated_target": validated if accepted else None,
        "controller_target_frame": "base_link",
        "projection_distance": float(selection.get("projection_distance_m", 0.0)),
        "acceptance_status": "target_accepted" if accepted else "target_unreachable",
        "orientation_used": False,
        "accepted_target_timestamp_s": (
            selection.get("accepted_target_timestamp_s") if accepted else None
        ),
        "reference_pose_count": 1 if accepted else 0,
        "seed": seed,
        "result_status": "passed" if orchestration.get("orchestration_success") is True else "failed",
    }


def collect_metrics(result: dict[str, Any]) -> dict[str, Any]:
    candidate_dir = Path(result["candidate_dir"])
    summary = _read_json(candidate_dir / "summary.json")
    orchestration = _read_json(candidate_dir / "orchestration.json")
    lumen = summary.get("lumen_evaluation", {})
    physical = lumen.get("physical_safety", {}) if isinstance(lumen, dict) else {}
    progress = lumen.get("progress", {}) if isinstance(lumen, dict) else {}
    timing = summary.get("timing", {})
    tracking = summary.get("tracking", {})
    goal = summary.get("goal", {})
    run_status = summary.get("run_status", {})
    topic_status = summary.get("topic_status", {})
    cleanup = orchestration.get("cleanup_audit", {})
    readiness = orchestration.get("readiness_diagnostics", {})
    created = readiness.get("monitor_created_monotonic")
    completed = readiness.get("stability_collection_end_monotonic")
    readiness_time = None
    if _finite_number(created) and _finite_number(completed) and float(completed) >= float(created):
        readiness_time = float(completed) - float(created)
    return {
        "comparison_valid": bool(result.get("comparison_valid", False)),
        "readiness_succeeded": bool(
            orchestration.get("initial_state_stability", {}).get("stable", False)
        ),
        "readiness_time_seconds": readiness_time,
        "command_message_count": int(
            orchestration.get("command_audit", {}).get("command_message_count", 0)
        ),
        "controller_update_count": int(
            orchestration.get("command_audit", {}).get("command_message_count", 0)
        ),
        "tip_sample_count": int(topic_status.get("/ctr/tip", {}).get("count", 0)),
        "aligned_tip_sample_count": int(summary.get("data_quality", {}).get("valid_aligned_sample_count", 0)),
        "final_tip_to_target_distance_m": goal.get("final_goal_error", tracking.get("final_error")),
        "tip_to_target_rmse_m": goal.get("tip_to_target_rmse_m", goal.get("rmse")),
        "minimum_wall_clearance_m": physical.get("minimum_physical_clearance_m"),
        "collision_count": physical.get("collision_event_count"),
        "centerline_tracking_rmse_m": progress.get("centerline_tracking_rmse_m"),
        "runtime_seconds": timing.get("experiment_wall_duration"),
        "controller_update_frequency_hz": timing.get("command_publication_rate"),
        "effective_solve_frequency_hz": timing.get("effective_solve_frequency"),
        "safety_events": summary.get("slice_7g_safety", {}).get("fault_count", 0),
        "tactile_invalid_events": summary.get("slice_7g_tactile", {}).get(
            "invalid_sample_count", 0
        ),
        "navigation_success": bool(summary.get("navigation", {}).get("navigation_success", False)),
        "goal_reached": bool(goal.get("goal_reached", False)),
        "run_status": run_status.get("status"),
        "cleanup_clean": bool(cleanup.get("clean", False)),
        "candidate_report": str(candidate_dir / "report.md"),
        "candidate_plots": [str(path) for path in sorted(candidate_dir.glob("*.png"))],
    }


def validate_functional_result(
    result: dict[str, Any],
    metrics: dict[str, Any],
    *,
    require_navigation: bool = True,
) -> str | None:
    checks = (
        (result.get("orchestration_success") is True, "orchestration did not complete"),
        (metrics["comparison_valid"] is True, "baseline/candidate comparison is invalid"),
        (metrics["readiness_succeeded"] is True, "readiness was not reached"),
        (metrics["command_message_count"] > 0, "controller produced no command messages"),
        (metrics["cleanup_clean"] is True, "owned process cleanup did not pass"),
        (metrics.get("run_status") == "completed", "evaluation window did not complete"),
        (
            not require_navigation or metrics.get("navigation_success") is True,
            "accepted target was not reached with valid collision-free navigation",
        ),
        (int(metrics.get("safety_events", 0)) == 0, "a safety fault occurred"),
    )
    for passed, reason in checks:
        if not passed:
            return reason
    return None


def write_development_results(root: Path, attempts: list[dict[str, Any]]) -> dict[str, str]:
    payload = {
        "schema_version": "ctr-slice-7g-development-simulation-results-2",
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "development_simulation": True,
        "simulator_only": True,
        "production_promotion_evidence": False,
        "production_attempts_consumed": 0,
        "disclaimer": DEVELOPMENT_SIMULATION_DISCLAIMER,
        "attempts": attempts,
    }
    json_path = root / "development_results.json"
    json_path.write_text(
        json.dumps(sanitize_for_json(payload), indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    plot_path = root / "seed_comparison.png"
    generate_seed_plot(attempts, plot_path)
    report_path = root / "development_report.md"
    report_path.write_text(development_report(root, attempts, plot_path), encoding="utf-8")
    return {
        "result_root": str(root),
        "result_report": str(report_path),
        "result_json": str(json_path),
        "comparison_plot": str(plot_path),
    }


def generate_seed_plot(attempts: list[dict[str, Any]], path: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    examples = [item for item in attempts if item["kind"] == "example"]
    labels = [str(item["seed"]) for item in examples]
    errors = [_plot_number(item["metrics"].get("final_tip_to_target_distance_m")) for item in examples]
    clearance = [_plot_number(item["metrics"].get("minimum_wall_clearance_m")) for item in examples]
    figure, axes = plt.subplots(1, 2, figsize=(9, 4))
    axes[0].bar(labels, errors, color="#3465a4")
    axes[0].set_title("Final tip-to-target distance")
    axes[0].set_xlabel("Seed")
    axes[0].set_ylabel("Distance (m)")
    axes[1].bar(labels, clearance, color="#4e9a06")
    axes[1].set_title("Minimum physical clearance")
    axes[1].set_xlabel("Seed")
    axes[1].set_ylabel("Clearance (m)")
    figure.suptitle("Slice 7G development simulation (not production evidence)")
    figure.tight_layout()
    figure.savefig(path, dpi=140)
    plt.close(figure)


def development_report(root: Path, attempts: list[dict[str, Any]], plot_path: Path) -> str:
    lines = [
        "# Slice 7G Development Simulation Results",
        "",
        f"> **{DEVELOPMENT_SIMULATION_DISCLAIMER}**",
        "",
        "The workflow used only the software simulator, simulated tactile input, and the safety supervisor.",
        "No production authority, budget, domain lease, evidence seal, or campaign attempt was used.",
        "",
        "## Results",
        "",
        "| Kind | Seed | Status | ROS domain | Readiness (s) | Final target error (m) | Tip-to-target RMSE (m) | Centerline RMSE (m) | Min clearance (m) | Collisions | Solve Hz | Safety/tactile events | Failure |",
        "| --- | ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |",
    ]
    for item in attempts:
        metrics = item["metrics"]
        lines.append(
            "| {kind} | {seed} | {status} | {domain} | {ready} | {final} | {rmse} | "
            "{centerline} | {clearance} | {collisions} | {frequency} | {safety}/{tactile} | {failure} |".format(
                kind=item["kind"],
                seed=item["seed"],
                status=item["status"],
                domain=_format(item.get("ros_domain_id")),
                ready=_format(metrics.get("readiness_time_seconds")),
                final=_format(metrics.get("final_tip_to_target_distance_m")),
                rmse=_format(metrics.get("tip_to_target_rmse_m")),
                centerline=_format(metrics.get("centerline_tracking_rmse_m")),
                clearance=_format(metrics.get("minimum_wall_clearance_m")),
                collisions=_format(metrics.get("collision_count")),
                frequency=_format(metrics.get("effective_solve_frequency_hz")),
                safety=_format(metrics.get("safety_events")),
                tactile=_format(metrics.get("tactile_invalid_events")),
                failure=item.get("failure_reason") or "",
            )
        )
    lines.extend(["", "## Plots", "", f"![Per-seed comparison]({plot_path.name})", ""])
    lines.extend(["", "## Target selection", ""])
    for item in attempts:
        target = item.get("target_selection", {})
        lines.append(
            "- {kind} seed {seed}: source=`{source}`, raw=`{raw}` in `{raw_frame}`, "
            "accepted=`{validated}` in `{controller_frame}`, projection=`{projection:.6g} m`, "
            "orientation_used=`{orientation}`, reference_pose_count=`{poses}`, status=`{status}`.".format(
                kind=item["kind"],
                seed=item["seed"],
                source=target.get("target_source", "unknown"),
                raw=target.get("raw_input_point"),
                raw_frame=target.get("raw_input_frame", "unknown"),
                validated=target.get("validated_target"),
                controller_frame=target.get("controller_target_frame", "unknown"),
                projection=float(target.get("projection_distance", 0.0)),
                orientation=target.get("orientation_used", False),
                poses=target.get("reference_pose_count", 0),
                status=target.get("acceptance_status", "unknown"),
            )
        )
    for item in attempts:
        candidate_dir = item.get("candidate_dir")
        if not candidate_dir:
            continue
        relative = Path(candidate_dir).relative_to(root)
        lines.append(f"- {item['kind']} seed {item['seed']}: [{relative}/report.md]({relative}/report.md)")
    lines.extend(
        [
            "",
            "## Visual simulation",
            "",
            "After sourcing ROS and this workspace install, run:",
            "",
            "```bash",
            "ROS_DOMAIN_ID=166 ros2 launch ctr_bringup slice_7g_development_visual.launch.py development_simulation:=true seed:=11",
            "```",
            "",
            "The RViz view shows the CTR/lumen marker array, reference path, and tip pose. The fixed domain is an example; choose another unused domain if 166 is occupied.",
            "",
            "## Known limitations",
            "",
            "- Results are software-simulation evidence only.",
            "- Controller timing is descriptive and Python MPPI may not be real-time capable.",
            "- Hardware, privileged installation, production cleanup authority, and physical validation remain untested.",
            "",
        ]
    )
    return "\n".join(lines)


def _result_root(override: str) -> Path:
    if override:
        candidate = Path(override)
    else:
        timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        candidate = Path.cwd() / "evaluation_results" / f"slice_7g_development_{timestamp}"
    return validate_development_output_root(candidate)


def _read_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if type(value) is not dict:
        raise OrchestrationError(f"expected JSON object: {path}")
    return value


def _positive_finite(value: Any) -> bool:
    return type(value) in (int, float) and not isinstance(value, bool) and math.isfinite(value) and value > 0


def _finite_number(value: Any) -> bool:
    return type(value) in (int, float) and not isinstance(value, bool) and math.isfinite(value)


def _plot_number(value: Any) -> float:
    return float(value) if _finite_number(value) else 0.0


def _format(value: Any) -> str:
    if _finite_number(value):
        return f"{float(value):.6g}"
    return "" if value is None else str(value)


if __name__ == "__main__":
    raise SystemExit(main())
