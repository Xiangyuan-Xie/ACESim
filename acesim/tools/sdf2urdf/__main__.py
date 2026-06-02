from __future__ import annotations

import argparse
import sys

from acesim.tools.sdf2urdf import (
    AssetPaths,
    AssetToolchainConfig,
    cleanup_manual_meshes_from_sdf,
    generate_manual_meshes_from_sdf,
    sync_manual_urdf_from_sdf,
)


def main(argv: list[str] | None = None) -> int:
    raw_argv = sys.argv[1:] if argv is None else argv
    parser = argparse.ArgumentParser(description="Synchronize ACESim URDF assets from an SDF source provider.")
    parser.add_argument("--tui", action="store_true", help="Launch the interactive terminal UI.")
    parser.add_argument("--source", default="px4", help="SDF source provider name.")
    parser.add_argument("--target", help="Asset target name.")
    parser.add_argument("--cleanup", action="store_true", help="Delete stale generated meshes after sync.")
    args = parser.parse_args(raw_argv)

    if args.tui or not raw_argv:
        from acesim.tools.sdf2urdf import tui

        return tui.main()

    if not args.target:
        parser.error("--target is required unless --tui is used")

    config = AssetToolchainConfig(target=args.target)
    paths = AssetPaths.for_target(args.target)
    if not paths.urdf_path.exists():
        print(f"Error: URDF not found at {paths.urdf_path}", file=sys.stderr)
        return 1

    try:
        generate_manual_meshes_from_sdf(config, paths, source=args.source)
        sync_manual_urdf_from_sdf(config, paths, source=args.source)
        if args.cleanup:
            cleanup_manual_meshes_from_sdf(config, paths, source=args.source)
    except Exception as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    print(paths.urdf_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
