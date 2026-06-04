from __future__ import annotations

import argparse
from pathlib import Path
from typing import Literal

from acesim.sitl.runner import PX4SITLConfig, PX4SITLRunner


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run ACESim core PX4 SITL without requiring ROS 2.")
    parser.add_argument("--px4-repo", type=Path, default=None, help="PX4-Autopilot repository path.")
    parser.add_argument("--config", dest="config_path", type=Path, default=None, help="ACESim config TOML path.")
    mode_group = parser.add_mutually_exclusive_group()
    mode_group.add_argument("--headless", action="store_true", help="Run MuJoCo without the viewer.")
    mode_group.add_argument("--gui", action="store_true", help="Run MuJoCo with the viewer.")
    parser.add_argument(
        "--readiness-mode",
        choices=["background", "wait", "off"],
        default="background",
        help="PX4 post-start readiness behavior.",
    )
    parser.add_argument(
        "--no-readiness-check",
        action="store_true",
        help="Compatibility alias for --readiness-mode off.",
    )
    parser.add_argument("--px4-instance", type=int, default=0, help="PX4 instance index for port isolation.")
    return parser


def config_from_args(args: argparse.Namespace) -> PX4SITLConfig:
    readiness_mode: Literal["background", "wait", "off"]
    if args.no_readiness_check:
        readiness_mode = "off"
    elif args.readiness_mode in {"background", "wait", "off"}:
        readiness_mode = args.readiness_mode
    else:
        raise ValueError("readiness_mode must be one of: background, wait, off")
    return PX4SITLConfig(
        px4_repo=args.px4_repo,
        config_path=args.config_path,
        headless=not args.gui,
        px4_instance=args.px4_instance,
        readiness_mode=readiness_mode,
    )


def main(argv: list[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)
    return PX4SITLRunner(config_from_args(args)).run()


if __name__ == "__main__":
    raise SystemExit(main())
