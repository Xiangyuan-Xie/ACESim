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


def main() -> int:
    parser = argparse.ArgumentParser(description="Synchronize ACESim URDF assets from an SDF source provider.")
    parser.add_argument("--source", default="px4", help="SDF source provider name.")
    parser.add_argument("--target", required=True, help="Asset target name.")
    parser.add_argument("--cleanup", action="store_true", help="Delete stale generated meshes after sync.")
    args = parser.parse_args()

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
