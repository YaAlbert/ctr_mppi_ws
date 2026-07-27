"""Offline Markdown and plot generation for evaluation runs."""

from __future__ import annotations

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
        "## Experiment Identity",
        "",
        f"- run_id: `{metadata.get('run_id', '')}`",
        f"- experiment_group: `{metadata.get('experiment_group', '')}`",
        f"- controller_label: `{metadata.get('controller_label', '')}`",
        f"- git_commit: `{metadata.get('git', {}).get('short_commit', '')}`",
        f"- git_branch: `{metadata.get('git', {}).get('branch', '')}`",
        f"- workspace_dirty: `{metadata.get('git', {}).get('dirty', '')}`",
        f"- ros_domain_id: `{metadata.get('ros_domain_id', '')}`",
        f"- configured_duration_s: `{metadata.get('configured_duration', '')}`",
        f"- actual_duration_s: `{metadata.get('actual_duration', '')}`",
        "",
        "## Configuration",
        "",
    ]
    configuration = metadata.get("configuration", {})
    for key in (
        "trajectory_type",
        "frame_id",
        "configured_control_period",
        "reference_sample_period",
        "software_mode",
    ):
        lines.append(f"- {key}: `{configuration.get(key, '')}`")
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

    lines.extend(["", "## Metrics", ""])
    lines.extend(_metrics_table("Tracking", summary.get("tracking", {})))
    lines.extend(_metrics_table("Control", summary.get("control", {})))
    lines.extend(_metrics_table("Timing", summary.get("timing", {})))
    lines.extend(_metrics_table("Data Quality", summary.get("data_quality", {})))
    lines.extend(_metrics_table("Numerical Safety", summary.get("numerical_safety", {})))

    lines.extend(["", "## Acceptance", "", "| Category | Pass |", "| --- | --- |"])
    for key, value in summary.get("acceptance", {}).items():
        if key == "reasons":
            continue
        lines.append(f"| {key} | {value} |")
    reasons = summary.get("acceptance", {}).get("reasons", [])
    if reasons:
        lines.extend(["", "Reasons:"])
        lines.extend(f"- {reason}" for reason in reasons)

    if comparison is not None:
        lines.extend(["", "## Baseline Comparison", ""])
        if not comparison.get("compatibility_valid", False):
            lines.append("Comparison is not compatibility-valid.")
            for reason in comparison.get("compatibility_reasons", []):
                lines.append(f"- {reason}")
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


def generate_plots(run_dir: Path, samples: list[AlignedSample]) -> list[Path]:
    paths = [
        run_dir / "tracking_error.png",
        run_dir / "trajectory_xy.png",
        run_dir / "trajectory_3d.png",
        run_dir / "command_history.png",
        run_dir / "solve_time.png",
        run_dir / "cumulative_control_effort.png",
    ]
    if not samples:
        for path in paths:
            _empty_plot(path, "No aligned samples")
        return paths

    times = np.asarray([sample.timestamp for sample in samples], dtype=float)
    times = times - times[0]
    tip = np.asarray([sample.tip_position for sample in samples], dtype=float)
    reference = np.asarray([sample.reference_position for sample in samples], dtype=float)
    commands = np.asarray([sample.command for sample in samples], dtype=float)
    errors = np.linalg.norm(tip - reference, axis=1)
    solve_times = np.asarray(
        [math.nan if not math.isfinite(sample.solve_time) else sample.solve_time for sample in samples],
        dtype=float,
    )
    effort = np.asarray(
        compute_control_effort_series(times=times, commands=commands).cumulative_total_effort,
        dtype=float,
    )

    _line_plot(paths[0], times, [errors], ["tip error"], "Tracking Error", "time [s]", "error [m]")
    _xy_plot(paths[1], tip, reference)
    _trajectory_3d_plot(paths[2], tip, reference)
    _line_plot(
        paths[3],
        times,
        [commands[:, index] for index in range(commands.shape[1])],
        [f"u{index}" for index in range(commands.shape[1])],
        "Command History",
        "time [s]",
        "command [SI units/s]",
    )
    _line_plot(paths[4], times, [solve_times], ["solve time"], "Solve Time", "time [s]", "solve time [s]")
    _line_plot(paths[5], times, [effort], ["effort"], "Cumulative Control Effort", "time [s]", "sum ||u||^2 dt")
    return paths


def _metrics_table(title: str, values: dict[str, Any]) -> list[str]:
    lines = ["", f"### {title}", "", "| Metric | Value |", "| --- | ---: |"]
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
