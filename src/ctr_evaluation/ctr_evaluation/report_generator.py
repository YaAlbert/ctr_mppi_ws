"""Offline Markdown and plot generation for evaluation runs."""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
import tempfile
from typing import Any

os.environ.setdefault("MPLCONFIGDIR", str(Path(tempfile.gettempdir()) / "ctr_mppi_matplotlib"))

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from ctr_evaluation.metrics import compute_control_effort_series
from ctr_evaluation.time_alignment import AlignedSample

TIMING_DESCRIPTION = (
    "Timing and solver-performance metrics are descriptive only and are not navigation acceptance criteria."
)


def generate_report(
    *,
    run_dir: Path,
    metadata: dict[str, Any],
    summary: dict[str, Any],
    comparison: dict[str, Any] | None,
    plot_paths: list[Path],
) -> Path:
    path = run_dir / "report.md"
    lines = [
        "# CTR Evaluation Report",
        "",
    ]
    if metadata.get("development_simulation") is True:
        lines.extend(
            [
                "> **Development simulation only.** This report is not production promotion evidence.",
                "",
            ]
        )
    lines.extend([
        "## Experiment Identity",
        "",
        f"- run_id: `{metadata.get('run_id', '')}`",
        f"- experiment_group: `{metadata.get('experiment_group', '')}`",
        f"- controller_label: `{metadata.get('controller_label', '')}`",
        f"- git_commit: `{metadata.get('git', {}).get('short_commit', '')}`",
        f"- git_branch: `{metadata.get('git', {}).get('branch', '')}`",
        f"- workspace_dirty: `{metadata.get('git', {}).get('dirty', '')}`",
        f"- ros_domain_id: `{metadata.get('ros_domain_id', '')}`",
        f"- recorder_default_configured_duration_s: `{metadata.get('configured_duration', '')}`",
        f"- requested_evaluation_duration_s: `{metadata.get('requested_evaluation_duration_s', '')}`",
        f"- evaluation_window_duration_s: `{metadata.get('evaluation_window_duration_s', '')}`",
        f"- actual_recording_duration_s: `{metadata.get('actual_duration', '')}`",
        "",
        "## Configuration",
        "",
    ])
    configuration = metadata.get("configuration", {})
    for key in (
        "trajectory_type",
        "frame_id",
        "configured_control_period",
        "reference_sample_period",
        "software_mode",
    ):
        lines.append(f"- {key}: `{configuration.get(key, '')}`")

    target_selection = metadata.get("development_target_selection")
    if isinstance(target_selection, dict):
        lines.extend(
            [
                "",
                "## Development Target Selection",
                "",
                f"- target_source: `{target_selection.get('target_source', '')}`",
                f"- raw_input_point_m: `{target_selection.get('raw_input_point', '')}`",
                f"- raw_input_frame: `{target_selection.get('raw_input_frame', '')}`",
                f"- validated_target_m: `{target_selection.get('validated_target', '')}`",
                f"- controller_target_frame: `{target_selection.get('controller_target_frame', '')}`",
                f"- projection_distance_m: `{target_selection.get('projection_distance_m', '')}`",
                f"- acceptance_status: `{target_selection.get('acceptance_status', '')}`",
                f"- orientation_used: `{target_selection.get('orientation_used', '')}`",
                f"- reference_pose_count: `{target_selection.get('reference_pose_count', '')}`",
            ]
        )
    lines.extend(
        [
            "",
            "## Topic Status",
            "",
            "| Topic | Required | Received | Count |",
            "| --- | --- | --- | ---: |",
        ]
    )
    for topic, values in sorted(metadata.get("topics", {}).items()):
        lines.append(
            f"| `{topic}` | {values.get('required', False)} | "
            f"{values.get('received', False)} | {values.get('count', 0)} |"
        )

    run_status = summary.get("run_status", {})
    lines.extend(
        [
            "",
            "## Completion Status",
            "",
            f"- status: `{run_status.get('status', 'unknown')}`",
            f"- interrupted: `{run_status.get('interrupted', False)}`",
            f"- completed_evaluation_window: `{run_status.get('completed_evaluation_window', False)}`",
        ]
    )
    lines.extend(["", "## Metrics", ""])
    tracking = dict(summary.get("tracking", {}))
    semantics = summary.get("metric_semantics", {})
    tracking_rmse_name = str(semantics.get("tracking_rmse_name", "reference_tracking_rmse_m"))
    if "rmse" in tracking:
        tracking[tracking_rmse_name] = tracking.pop("rmse")
    lines.extend(_metrics_table("Tracking", tracking))
    lines.extend(_metrics_table("Control", summary.get("control", {})))
    goal = dict(summary.get("goal", {}))
    goal.pop("rmse", None)
    lines.extend(_metrics_table("Goal", goal))
    lines.extend(_metrics_table("Lumen Safety", summary.get("lumen_safety", {})))
    lines.extend(_metrics_table("Motion", summary.get("motion", {})))
    lines.extend(_metrics_table("Timing (Descriptive)", summary.get("timing", {}), description=TIMING_DESCRIPTION))
    lines.extend(_metrics_table("Data Quality", summary.get("data_quality", {})))
    lines.extend(_metrics_table("Numerical Safety", summary.get("numerical_safety", {})))
    lines.extend(
        [
            "",
            "### Metric Semantics",
            "",
            f"- `{tracking_rmse_name}`: `{semantics.get('tracking_rmse_formula', '')}` in metres.",
            "- `final_goal_error`: Euclidean distance from the final aligned tip sample to the accepted target, in metres.",
            "- `centerline_tracking_rmse_m`: RMS closest-centerline radial offset over aligned tip samples, in metres.",
        ]
    )

    lines.extend(["", "## Acceptance", "", "| Category | Pass |", "| --- | --- |"])
    for key, value in summary.get("acceptance", {}).items():
        if key in {"reasons", "timing_pass", "real_time_pass"}:
            continue
        lines.append(f"| {key} | {value} |")
    reasons = summary.get("acceptance", {}).get("reasons", [])
    if reasons:
        lines.extend(["", "Reasons:"])
        lines.extend(f"- {reason}" for reason in reasons)

    if comparison is not None:
        lines.extend(["", "## Baseline Comparison", ""])
        comparison_valid = bool(comparison.get("comparison_valid", comparison.get("compatibility_valid", False)))
        if not comparison.get("compatibility_valid", False):
            lines.append("Comparison is not compatibility-valid.")
            for reason in comparison.get("compatibility_reasons", []):
                lines.append(f"- {reason}")
        if not comparison_valid:
            lines.append("Comparison is not valid; improvement was not evaluated.")
        details = comparison.get("compatibility_details", {})
        if details:
            lines.extend(["", "Compatibility details:"])
            for key, value in sorted(details.items()):
                lines.append(f"- {key}: `{_fmt(value)}`")
        lines.extend(["", "| Metric | Candidate | Baseline | Difference | Relative improvement % | Valid |", "| --- | ---: | ---: | ---: | ---: | --- |"])
        for item in comparison.get("metric_comparisons", []):
            lines.append(
                f"| {item['metric']} | {_fmt(item['candidate_value'])} | {_fmt(item['baseline_value'])} | "
                f"{_fmt(item['absolute_difference'])} | {_fmt(item.get('relative_improvement_percent'))} | "
                f"{item['comparison_valid']} |"
            )

    lines.extend(["", "## Plots And Files", ""])
    for plot_path in sorted(plot_paths):
        lines.append(f"- `{plot_path.name}`")
    for data_path in sorted(run_dir.glob("*.csv")):
        lines.append(f"- `{data_path.name}`")
    for json_path in sorted(run_dir.glob("*.json")):
        lines.append(f"- `{json_path.name}`")

    lines.extend(
        [
            "",
            "## Warnings And Limitations",
            "",
            "- This evaluator uses state timestamps and immediate-reference interpolation.",
            "- Command timestamps are command-publication times, not proven application times.",
            "- Horizon paths do not contain per-horizon-point timestamps.",
            "- Physical validation and hardware validation remain false unless separately verified.",
        ]
    )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


_BASE_PLOT_ARTIFACTS = (
    ("tracking_error_plot", "tracking_error.png"),
    ("trajectory_xy_plot", "trajectory_xy.png"),
    ("trajectory_3d_plot", "trajectory_3d.png"),
    ("tip_trajectory_plot", "tip_trajectory.png"),
    ("command_history_plot", "command_history.png"),
    ("solve_time_plot", "solve_time.png"),
    ("cumulative_control_effort_plot", "cumulative_control_effort.png"),
)

_CURVED_LUMEN_PLOT_ARTIFACTS = (
    ("curved_wall_clearance_plot", "curved_wall_clearance.png"),
    ("centerline_tracking_error_plot", "centerline_tracking_error.png"),
    ("curved_lumen_trajectory_plot", "curved_lumen_trajectory_3d.png"),
)


def plot_artifact_names(
    run_dir: Path,
    metadata: dict[str, Any] | None = None,
    *,
    include_cylinder_plots: bool | None = None,
    include_lumen_plots: bool | None = None,
) -> tuple[str, ...]:
    names = [name for name, _ in _BASE_PLOT_ARTIFACTS]
    if include_cylinder_plots is None:
        configuration = (
            {} if metadata is None else metadata.get("configuration", {})
        )
        lumen = configuration.get("cylindrical_lumen")
        include_cylinder_plots = (
            isinstance(lumen, dict)
            and (run_dir / "cylinder_navigation.csv").is_file()
        )
    elif type(include_cylinder_plots) is not bool:
        raise TypeError("include_cylinder_plots must be a bool or None")
    if include_cylinder_plots:
        names.extend(("wall_clearance_plot", "cylinder_backbone_target_plot"))
    if include_lumen_plots is None:
        include_lumen_plots = (run_dir / "lumen_evaluation.csv").is_file()
    elif type(include_lumen_plots) is not bool:
        raise TypeError("include_lumen_plots must be a bool or None")
    if include_lumen_plots:
        names.extend(name for name, _path in _CURVED_LUMEN_PLOT_ARTIFACTS)
    return tuple(names)


def generate_plot_artifact(
    logical_name: str,
    run_dir: Path,
    samples: list[AlignedSample],
    metadata: dict[str, Any] | None = None,
) -> Path:
    paths = dict((name, run_dir / path) for name, path in _BASE_PLOT_ARTIFACTS)
    curved_paths = dict(
        (name, run_dir / path) for name, path in _CURVED_LUMEN_PLOT_ARTIFACTS
    )
    if logical_name in curved_paths:
        path = curved_paths[logical_name]
        if logical_name == "curved_wall_clearance_plot":
            _curved_wall_clearance_plot(path, run_dir / "lumen_evaluation.csv")
        elif logical_name == "centerline_tracking_error_plot":
            _centerline_tracking_error_plot(path, run_dir / "lumen_evaluation.csv")
        else:
            _curved_lumen_trajectory_plot(path, run_dir, samples)
        return path
    if logical_name in {
        "wall_clearance_plot",
        "cylinder_backbone_target_plot",
    }:
        configuration = (
            {} if metadata is None else metadata.get("configuration", {})
        )
        lumen = configuration.get("cylindrical_lumen")
        cylinder_csv = run_dir / "cylinder_navigation.csv"
        if not isinstance(lumen, dict) or not cylinder_csv.is_file():
            raise FileNotFoundError(
                "cylinder plot prerequisites are unavailable"
            )
        tip = (
            np.asarray(
                [sample.tip_position for sample in samples], dtype=float
            )
            if samples
            else None
        )
        if logical_name == "wall_clearance_plot":
            path = run_dir / "wall_clearance.png"
            _wall_clearance_plot(path, cylinder_csv)
            return path
        path = run_dir / "cylinder_backbone_target_3d.png"
        _cylinder_3d_plot(
            path,
            run_dir,
            lumen,
            configuration.get("goal", {}),
            tip=tip,
        )
        return path
    if logical_name not in paths:
        raise KeyError(f"unknown plot artifact: {logical_name}")
    path = paths[logical_name]
    if not samples:
        _empty_plot(path, "No aligned samples")
        return path
    times = np.asarray([sample.timestamp for sample in samples], dtype=float)
    times = times - times[0]
    tip = np.asarray(
        [sample.tip_position for sample in samples], dtype=float
    )
    reference = np.asarray(
        [sample.reference_position for sample in samples], dtype=float
    )
    commands = np.asarray([sample.command for sample in samples], dtype=float)
    errors = np.linalg.norm(tip - reference, axis=1)
    solve_times = np.asarray(
        [
            math.nan if not math.isfinite(sample.solve_time)
            else sample.solve_time
            for sample in samples
        ],
        dtype=float,
    )
    effort = np.asarray(
        compute_control_effort_series(
            times=times,
            commands=commands,
        ).cumulative_total_effort,
        dtype=float,
    )
    if logical_name == "tracking_error_plot":
        fixed_target = _is_fixed_target_metadata(metadata)
        _line_plot(
            path,
            times,
            [errors],
            ["tip-to-target error" if fixed_target else "tip-to-reference error"],
            "Tip-To-Target Error" if fixed_target else "Reference Tracking Error",
            "time [s]",
            "error [m]",
        )
    elif logical_name == "trajectory_xy_plot":
        _xy_plot(path, tip, reference)
    elif logical_name in {"trajectory_3d_plot", "tip_trajectory_plot"}:
        _trajectory_3d_plot(path, tip, reference)
    elif logical_name == "command_history_plot":
        _line_plot(
            path,
            times,
            [commands[:, index] for index in range(commands.shape[1])],
            [f"u{index}" for index in range(commands.shape[1])],
            "Command History",
            "time [s]",
            "command [SI units/s]",
        )
    elif logical_name == "solve_time_plot":
        _line_plot(
            path,
            times,
            [solve_times],
            ["solve time"],
            "Solve Time",
            "time [s]",
            "solve time [s]",
        )
    else:
        _line_plot(
            path,
            times,
            [effort],
            ["effort"],
            "Cumulative Control Effort",
            "time [s]",
            "sum ||u||^2 dt",
        )
    return path


def plot_producer_registry(
    run_dir: Path,
    samples: list[AlignedSample],
    metadata: dict[str, Any] | None = None,
    *,
    include_cylinder_plots: bool | None = None,
    include_lumen_plots: bool | None = None,
) -> dict[str, Any]:
    return {
        name: (
            lambda output_dir, name=name: generate_plot_artifact(
                name, output_dir, samples, metadata
            )
        )
        for name in plot_artifact_names(
            run_dir,
            metadata,
            include_cylinder_plots=include_cylinder_plots,
            include_lumen_plots=include_lumen_plots,
        )
    }


def generate_plots(
    run_dir: Path,
    samples: list[AlignedSample],
    metadata: dict[str, Any] | None = None,
) -> list[Path]:
    registry = plot_producer_registry(run_dir, samples, metadata)
    return [
        registry[name](run_dir)
        for name in plot_artifact_names(run_dir, metadata)
    ]


def _is_fixed_target_metadata(metadata: dict[str, Any] | None) -> bool:
    if not isinstance(metadata, dict):
        return False
    if metadata.get("target_mode") == "fixed_target":
        return True
    override = metadata.get("metadata_override")
    return isinstance(override, dict) and override.get("target_mode") == "fixed_target"


def _curved_wall_clearance_plot(path: Path, csv_path: Path) -> None:
    data = _csv_numeric_columns(csv_path)
    times = data.get("timestamp_s", np.asarray([], dtype=float))
    clearance = data.get("physical_clearance_m", np.asarray([], dtype=float))
    if times.size == 0 or clearance.size == 0:
        _empty_plot(path, "No curved-lumen clearance samples")
        return
    _line_plot(
        path,
        times - times[0],
        [clearance],
        ["minimum backbone-to-wall clearance"],
        "Curved-Lumen Wall Clearance",
        "time [s]",
        "clearance [m]",
    )


def _centerline_tracking_error_plot(path: Path, csv_path: Path) -> None:
    data = _csv_numeric_columns(csv_path)
    times = data.get("timestamp_s", np.asarray([], dtype=float))
    offset = data.get("radial_offset_m", np.asarray([], dtype=float))
    if times.size == 0 or offset.size == 0:
        _empty_plot(path, "No centerline-deviation samples")
        return
    _line_plot(
        path,
        times - times[0],
        [offset],
        ["closest-centerline radial offset"],
        "Centerline Tracking Deviation",
        "time [s]",
        "deviation [m]",
    )


def _curved_lumen_trajectory_plot(
    path: Path,
    run_dir: Path,
    samples: list[AlignedSample],
) -> None:
    try:
        summary = json.loads((run_dir / "summary.json").read_text(encoding="utf-8"))
        lumen = summary["lumen_evaluation"]
        payload = lumen["geometry"]["fingerprint_payload"]
        centerline = np.asarray(payload["centerline_points"], dtype=float)
        radii = np.asarray(payload["lumen_radius"], dtype=float)
        target = np.asarray(lumen["identity"]["executed_target"], dtype=float)
    except (KeyError, OSError, TypeError, ValueError, json.JSONDecodeError):
        _empty_plot(path, "Curved-lumen geometry metadata unavailable")
        return
    tip = np.asarray([sample.tip_position for sample in samples], dtype=float)
    if (
        centerline.ndim != 2
        or centerline.shape[1:] != (3,)
        or radii.shape != (centerline.shape[0],)
        or tip.ndim != 2
        or tip.shape[1:] != (3,)
        or target.shape != (3,)
        or not all(np.all(np.isfinite(item)) for item in (centerline, radii, tip, target))
    ):
        _empty_plot(path, "Curved-lumen trajectory data invalid")
        return
    figure = plt.figure(figsize=(7, 5.5))
    axes = figure.add_subplot(111, projection="3d")
    axes.plot(centerline[:, 0], centerline[:, 1], centerline[:, 2], color="#55aaff", label="analytic centerline")
    axes.plot(tip[:, 0], tip[:, 1], tip[:, 2], color="#22aa44", label="actual tip trajectory")
    axes.scatter([target[0]], [target[1]], [target[2]], color="#f5c542", s=45, label="accepted target")
    for index in np.linspace(0, centerline.shape[0] - 1, min(14, centerline.shape[0]), dtype=int):
        tangent = (
            centerline[min(index + 1, centerline.shape[0] - 1)]
            - centerline[max(index - 1, 0)]
        )
        norm = float(np.linalg.norm(tangent))
        if norm <= 0.0:
            continue
        tangent /= norm
        helper = np.array([0.0, 1.0, 0.0])
        if abs(float(np.dot(helper, tangent))) > 0.9:
            helper = np.array([1.0, 0.0, 0.0])
        first = np.cross(tangent, helper)
        first /= np.linalg.norm(first)
        second = np.cross(tangent, first)
        angles = np.linspace(0.0, 2.0 * math.pi, 32)
        ring = centerline[index] + radii[index] * (
            np.cos(angles)[:, None] * first + np.sin(angles)[:, None] * second
        )
        axes.plot(ring[:, 0], ring[:, 1], ring[:, 2], color="0.65", alpha=0.25)
    axes.set_title("Actual Tip Trajectory In Analytic Curved Lumen")
    axes.set_xlabel("x [m]")
    axes.set_ylabel("y [m]")
    axes.set_zlabel("z [m]")
    axes.legend(loc="best")
    figure.tight_layout()
    figure.savefig(path)
    plt.close(figure)


def _metrics_table(title: str, values: dict[str, Any], description: str | None = None) -> list[str]:
    lines = ["", f"### {title}", "", "| Metric | Value |", "| --- | ---: |"]
    if description is not None:
        lines = ["", f"### {title}", "", description, "", "| Metric | Value |", "| --- | ---: |"]
    for key, value in sorted(values.items()):
        if isinstance(value, list):
            rendered = ", ".join(_fmt(item) for item in value)
        else:
            rendered = _fmt(value)
        lines.append(f"| {key} | {rendered} |")
    return lines


def _line_plot(path: Path, x: np.ndarray, series: list[np.ndarray], labels: list[str], title: str, xlabel: str, ylabel: str) -> None:
    fig, ax = plt.subplots(figsize=(8, 4.5))
    for y, label in zip(series, labels):
        ax.plot(x, y, label=label)
    ax.set_title(title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    if len(series) > 1:
        ax.legend(loc="best")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def _xy_plot(path: Path, tip: np.ndarray, reference: np.ndarray) -> None:
    fig, ax = plt.subplots(figsize=(5.5, 5.5))
    ax.plot(reference[:, 0], reference[:, 1], label="reference")
    ax.plot(tip[:, 0], tip[:, 1], label="tip")
    ax.set_title("Tip Trajectory XY")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.axis("equal")
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def _trajectory_3d_plot(path: Path, tip: np.ndarray, reference: np.ndarray) -> None:
    fig = plt.figure(figsize=(6, 5))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(reference[:, 0], reference[:, 1], reference[:, 2], label="reference")
    ax.plot(tip[:, 0], tip[:, 1], tip[:, 2], label="tip")
    ax.set_title("3D Tip Trajectory")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def _maybe_add_cylinder_plots(
    run_dir: Path,
    paths: list[Path],
    metadata: dict[str, Any] | None,
    *,
    tip: np.ndarray | None = None,
) -> list[Path]:
    configuration = {} if metadata is None else metadata.get("configuration", {})
    lumen = configuration.get("cylindrical_lumen")
    goal = configuration.get("goal", {})
    cylinder_csv = run_dir / "cylinder_navigation.csv"
    if not isinstance(lumen, dict) or not cylinder_csv.is_file():
        return paths
    wall_clearance_path = run_dir / "wall_clearance.png"
    cylinder_3d_path = run_dir / "cylinder_backbone_target_3d.png"
    _wall_clearance_plot(wall_clearance_path, cylinder_csv)
    _cylinder_3d_plot(cylinder_3d_path, run_dir, lumen, goal, tip=tip)
    return paths + [wall_clearance_path, cylinder_3d_path]


def _wall_clearance_plot(path: Path, csv_path: Path) -> None:
    data = _csv_numeric_columns(csv_path)
    times = data.get("timestamp", np.asarray([], dtype=float))
    clearance = data.get("minimum_backbone_clearance", np.asarray([], dtype=float))
    if times.size == 0 or clearance.size == 0:
        _empty_plot(path, "No cylinder clearance samples")
        return
    times = times - times[0]
    _line_plot(path, times, [clearance], ["minimum clearance"], "Wall Clearance", "time [s]", "clearance [m]")


def _cylinder_3d_plot(
    path: Path,
    run_dir: Path,
    lumen: dict[str, Any],
    goal: dict[str, Any],
    *,
    tip: np.ndarray | None,
) -> None:
    try:
        axis_origin = np.asarray(lumen["axis_origin"], dtype=float)
        axis_direction = np.asarray(lumen["axis_direction"], dtype=float)
        axis_direction = axis_direction / np.linalg.norm(axis_direction)
        radius = float(lumen["radius"])
        length = float(lumen["length"])
    except (KeyError, TypeError, ValueError):
        _empty_plot(path, "Malformed cylinder metadata")
        return
    fig = plt.figure(figsize=(6, 5))
    ax = fig.add_subplot(111, projection="3d")
    _plot_cylinder_wire(ax, axis_origin, axis_direction, radius, length)
    if tip is not None and tip.size:
        ax.plot(tip[:, 0], tip[:, 1], tip[:, 2], label="tip")
    backbone = _latest_backbone(run_dir / "backbone.csv")
    if backbone.size:
        ax.plot(backbone[:, 0], backbone[:, 1], backbone[:, 2], marker="o", markersize=2, label="final backbone")
    goal_position = goal.get("position") if isinstance(goal, dict) else None
    if goal_position is not None:
        target = np.asarray(goal_position, dtype=float)
        if target.shape == (3,) and np.all(np.isfinite(target)):
            ax.scatter([target[0]], [target[1]], [target[2]], s=40, label="goal")
    ax.set_title("Cylinder, Backbone, And Target")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.legend(loc="best")
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def _plot_cylinder_wire(ax, origin: np.ndarray, direction: np.ndarray, radius: float, length: float) -> None:
    helper = np.array([1.0, 0.0, 0.0], dtype=float)
    if abs(float(np.dot(helper, direction))) > 0.9:
        helper = np.array([0.0, 1.0, 0.0], dtype=float)
    u = np.cross(direction, helper)
    u = u / np.linalg.norm(u)
    v = np.cross(direction, u)
    theta = np.linspace(0.0, 2.0 * math.pi, 48)
    for axial in (0.0, length):
        center = origin + axial * direction
        ring = center[None, :] + radius * (np.cos(theta)[:, None] * u[None, :] + np.sin(theta)[:, None] * v[None, :])
        ax.plot(ring[:, 0], ring[:, 1], ring[:, 2], color="0.6", alpha=0.5)
    for angle in np.linspace(0.0, 2.0 * math.pi, 8, endpoint=False):
        radial = radius * (math.cos(angle) * u + math.sin(angle) * v)
        line = np.vstack((origin + radial, origin + length * direction + radial))
        ax.plot(line[:, 0], line[:, 1], line[:, 2], color="0.6", alpha=0.25)


def _latest_backbone(path: Path) -> np.ndarray:
    data = _csv_rows(path)
    if not data:
        return np.empty((0, 3), dtype=float)
    timed_rows = [row for row in data if row.get("timestamp") not in {None, ""}]
    if not timed_rows:
        return np.empty((0, 3), dtype=float)
    latest = max(float(row["timestamp"]) for row in timed_rows)
    rows = [row for row in timed_rows if float(row["timestamp"]) == latest]
    rows.sort(key=lambda row: int(float(row.get("index", 0))))
    points = [[float(row["x"]), float(row["y"]), float(row["z"])] for row in rows]
    return np.asarray(points, dtype=float)


def _csv_numeric_columns(path: Path) -> dict[str, np.ndarray]:
    rows = _csv_rows(path)
    if not rows:
        return {}
    result: dict[str, list[float]] = {key: [] for key in rows[0]}
    for row in rows:
        for key, value in row.items():
            try:
                result.setdefault(key, []).append(float(value))
            except (TypeError, ValueError):
                result.setdefault(key, []).append(math.nan)
    return {key: np.asarray(values, dtype=float) for key, values in result.items()}


def _csv_rows(path: Path) -> list[dict[str, str]]:
    if not path.is_file():
        return []
    import csv

    with path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def _empty_plot(path: Path, message: str) -> None:
    fig, ax = plt.subplots(figsize=(5, 3))
    ax.text(0.5, 0.5, message, ha="center", va="center")
    ax.set_axis_off()
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def _fmt(value: Any) -> str:
    if value is None:
        return "n/a"
    if isinstance(value, bool):
        return str(value)
    if isinstance(value, (int, float)):
        numeric = float(value)
        if not math.isfinite(numeric):
            return "nan"
        return f"{numeric:.6g}"
    return str(value)
