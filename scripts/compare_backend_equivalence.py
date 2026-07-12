#!/usr/bin/env python3
"""Compare traces emitted by the original and refactored SUPER backends."""

import math
import re
import sys
from pathlib import Path


FLOAT = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"


def vector(text: str, tag: str):
    match = re.search(rf"^\[{tag}\]\s+(.+)$", text, re.MULTILINE)
    if not match:
        raise AssertionError(f"missing [{tag}]")
    return [float(value) for value in re.findall(FLOAT, match.group(1))]


def scalar(text: str, pattern: str, name: str):
    match = re.search(pattern, text, re.MULTILINE)
    if not match:
        raise AssertionError(f"missing {name}")
    return float(match.group(1))


def parse(path: Path):
    text = path.read_text(errors="replace")
    samples = []
    for line in text.splitlines():
        if "[EQ-SAMPLE]" not in line:
            continue
        values = [float(value) for value in re.findall(FLOAT, line)]
        samples.append(values)
    return {
        "x": vector(text, "EQ-X"),
        "g": vector(text, "EQ-G"),
        "cost": scalar(text, rf"^\[EQ-FIRST\] cost=({FLOAT})", "first cost"),
        "iterations": int(scalar(text, r"Opt finish, with iter num:\s*(\d+)", "iteration count")),
        "duration": scalar(text, rf"^\[EQ-RESULT\].*duration=({FLOAT})", "duration"),
        "samples": samples,
    }


def norm(values):
    return math.sqrt(sum(value * value for value in values))


def relative_vector_error(lhs, rhs):
    if len(lhs) != len(rhs):
        raise AssertionError(f"vector sizes differ: {len(lhs)} != {len(rhs)}")
    delta = norm([a - b for a, b in zip(lhs, rhs)])
    return delta / max(1.0, norm(lhs), norm(rhs))


def main():
    if len(sys.argv) not in (3, 4):
        raise SystemExit(
            "usage: compare_backend_equivalence.py ORIGINAL_LOG REFACTORED_LOG [--objective-only]"
        )
    objective_only = len(sys.argv) == 4 and sys.argv[3] == "--objective-only"
    if len(sys.argv) == 4 and not objective_only:
        raise SystemExit(f"unknown option: {sys.argv[3]}")
    original = parse(Path(sys.argv[1]))
    refactored = parse(Path(sys.argv[2]))

    x_error = relative_vector_error(original["x"], refactored["x"])
    g_error = relative_vector_error(original["g"], refactored["g"])
    cost_error = abs(original["cost"] - refactored["cost"]) / max(1.0, abs(original["cost"]))
    duration_error = abs(original["duration"] - refactored["duration"])
    sample_error = relative_vector_error(
        [value for sample in original["samples"] for value in sample],
        [value for sample in refactored["samples"] for value in sample],
    )
    iteration_delta = abs(original["iterations"] - refactored["iterations"])

    assert x_error <= 1e-14, f"decision variables differ: {x_error}"
    assert cost_error <= 1e-13, f"first costs differ: {cost_error}"
    assert g_error <= 1e-12, f"first gradients differ: {g_error}"
    if not objective_only:
        assert duration_error <= 1e-4, f"durations differ: {duration_error}"
        assert sample_error <= 1e-4, f"trajectory samples differ: {sample_error}"
        assert iteration_delta <= 10, f"LBFGS evaluation counts differ: {iteration_delta}"

    print(
        f"backend {'objective' if objective_only else 'full'} equivalence passed: "
        f"x_rel={x_error:.3e} cost_rel={cost_error:.3e} "
        f"grad_rel={g_error:.3e} duration_abs={duration_error:.3e} "
        f"sample_rel={sample_error:.3e} iteration_delta={iteration_delta}"
    )


if __name__ == "__main__":
    main()
