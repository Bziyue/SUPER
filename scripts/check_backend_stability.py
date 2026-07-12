#!/usr/bin/env python3

import math
import re
import subprocess
import sys
from pathlib import Path


def parse_scalar(output: str, pattern: str, label: str) -> float:
    match = re.search(pattern, output)
    if match is None:
        raise AssertionError(f"missing {label} in backend output")
    return float(match.group(1))


def main() -> None:
    if len(sys.argv) not in (2, 3):
        raise SystemExit(
            "usage: check_backend_stability.py BACKEND_EQUIVALENCE [OPT_ACCURACY]"
        )

    executable = Path(sys.argv[1]).resolve()
    accuracy = sys.argv[2] if len(sys.argv) == 3 else "5e-6"
    max_callbacks = 400
    max_omega_excess = 0.01
    max_omega = 5.0
    max_position_violation = 0.2

    for perturbation in ("-1", "0", "1"):
        result = subprocess.run(
            [str(executable), perturbation, accuracy],
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        if result.returncode != 0:
            raise AssertionError(
                f"perturbation {perturbation} failed with {result.returncode}\n"
                f"{result.stdout}"
            )

        callbacks = int(parse_scalar(result.stdout, r"iter num: ([0-9]+)", "callback count"))
        squared_violation = parse_scalar(
            result.stdout, r"\bOmg: ([0-9.eE+\-]+)", "angular-rate violation"
        )
        position_violation = parse_scalar(
            result.stdout, r"\bPos: ([0-9.eE+\-]+)", "position violation"
        )
        omega_excess = math.sqrt(max_omega * max_omega + max(0.0, squared_violation)) - max_omega

        if callbacks > max_callbacks:
            raise AssertionError(
                f"perturbation {perturbation}: {callbacks} callbacks exceed {max_callbacks}"
            )
        if omega_excess > max_omega_excess:
            raise AssertionError(
                f"perturbation {perturbation}: omega excess {omega_excess} exceeds "
                f"{max_omega_excess} rad/s"
            )
        if position_violation > max_position_violation:
            raise AssertionError(
                f"perturbation {perturbation}: position violation "
                f"{position_violation} exceeds {max_position_violation} m"
            )

        print(
            f"perturbation={perturbation} callbacks={callbacks} "
            f"position_violation={position_violation:.6e} m "
            f"omega_excess={omega_excess:.6e} rad/s"
        )


if __name__ == "__main__":
    main()
