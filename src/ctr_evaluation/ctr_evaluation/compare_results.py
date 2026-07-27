"""Compare saved evaluation result directories."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import yaml

from ctr_evaluation.metrics import compare_summaries, sanitize_for_json


def compare_result_dirs(
    *,
    candidate_dir: Path,
    baseline_dir: Path,
    duration_tolerance: float,
    initial_state_tolerance: float,
    near_zero_epsilon: float,
) -> dict[str, Any]:
    candidate_summary = read_json(candidate_dir / "summary.json")
    baseline_summary = read_json(baseline_dir / "summary.json")
    candidate_metadata = read_yaml(candidate_dir / "metadata.yaml")
    baseline_metadata = read_yaml(baseline_dir / "metadata.yaml")
    result = compare_summaries(
        candidate_summary=candidate_summary,
        baseline_summary=baseline_summary,
        candidate_metadata=candidate_metadata,
        baseline_metadata=baseline_metadata,
        duration_tolerance=duration_tolerance,
        initial_state_tolerance=initial_state_tolerance,
        near_zero_epsilon=near_zero_epsilon,
    ).to_dict()
    write_json(candidate_dir / "comparison.json", result)
    write_comparison_markdown(candidate_dir / "comparison.md", result, candidate_dir, baseline_dir)
    return result


def write_comparison_markdown(path: Path, comparison: dict[str, Any], candidate_dir: Path, baseline_dir: Path) -> None:
    lines = [
        "# Evaluation Baseline Comparison",
        "",
        f"- candidate: `{candidate_dir}`",
        f"- baseline: `{baseline_dir}`",
        f"- compatibility_valid: `{comparison.get('compatibility_valid', False)}`",
        "",
    ]
    reasons = comparison.get("compatibility_reasons", [])
    if reasons:
        lines.append("Compatibility reasons:")
        lines.extend(f"- {reason}" for reason in reasons)
        lines.append("")
    lines.extend(
        [
            "| Metric | Direction | Candidate | Baseline | Difference | Relative improvement % | Valid | Reason |",
            "| --- | --- | ---: | ---: | ---: | ---: | --- | --- |",
        ]
    )
    for item in comparison.get("metric_comparisons", []):
        lines.append(
            f"| {item['metric']} | {item['direction']} | {item['candidate_value']:.6g} | "
            f"{item['baseline_value']:.6g} | {item['absolute_difference']:.6g} | "
            f"{_optional_number(item.get('relative_improvement_percent'))} | "
            f"{item['comparison_valid']} | {item['reason']} |"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def read_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def read_yaml(path: Path) -> dict[str, Any]:
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(data, dict):
        raise ValueError(f"YAML file must contain a map: {path}")
    return data


def write_json(path: Path, data: Any) -> None:
    path.write_text(json.dumps(sanitize_for_json(data), indent=2, allow_nan=False) + "\n", encoding="utf-8")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Compare CTR evaluation result directories.")
    parser.add_argument("candidate_dir", type=Path)
    parser.add_argument("baseline_dir", type=Path)
    parser.add_argument("--duration-tolerance", type=float, default=1.0)
    parser.add_argument("--initial-state-tolerance", type=float, default=1.0e-6)
    parser.add_argument("--near-zero-epsilon", type=float, default=1.0e-12)
    args = parser.parse_args(argv)
    compare_result_dirs(
        candidate_dir=args.candidate_dir,
        baseline_dir=args.baseline_dir,
        duration_tolerance=args.duration_tolerance,
        initial_state_tolerance=args.initial_state_tolerance,
        near_zero_epsilon=args.near_zero_epsilon,
    )
    return 0


def _optional_number(value) -> str:
    if value is None:
        return "n/a"
    return f"{float(value):.6g}"


if __name__ == "__main__":
    raise SystemExit(main())
