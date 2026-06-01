# ACESim Agent Guide

This guide is for coding agents and maintainers working in the ACESim repository.
It summarizes the project shape, common commands, and local conventions so a new
agent can make focused changes without rediscovering the basics.

## Project Snapshot

ACESim is a Python simulation toolkit for vehicle systems. The current codebase
centers on MuJoCo and Genesis backends, PX4/ROS 2 integration, aerial manipulator
workflows, and asset conversion utilities.

Core capabilities:

- MuJoCo and Genesis simulation backends.
- Environment types: `mc`, `am`, `fw`, `vtol`, and `uuv`.
- Bundled assets: `iris`, `x500`, `x500_arm2x`, `typhoon_h480`,
  `advanced_plane`, `standard_vtol`, and `uuv_bluerov2_heavy`.
- Optional PX4 and ROS 2 deployment paths.
- SDF-to-URDF and URDF-to-MJCF asset tooling.

The root package metadata is defined in both `pyproject.toml` and `setup.py`.
Keep those two files aligned when changing packaging metadata or dependencies.

## Repository Map

- `acesim/core/` - Python entry points for local play and benchmark flows.
- `acesim/env/` - backend-specific simulation environments and MuJoCo assets.
- `acesim/config/` - default and backend-specific TOML configuration.
- `acesim/utils/` - shared runtime utilities, PX4 transport, stream encoding,
  frame math, and dynamics helpers.
- `acesim/tools/sdf2urdf/` - first-stage SDF source synchronization into local
  URDF assets.
- `acesim/tools/urdf2mjcf/` - second-stage URDF consumption and MJCF generation.
- `acesim/tools/render_readme_assets.py` - README gallery renderer.
- `acesim/deploy/aircraft/acesim_ros2/` - ROS 2 package, launch files, bridge
  plugins, and benchmark console scripts.
- `acesim/deploy/aircraft/px4_msgs/` - vendored ROS 2 message/service definitions.
- `acesim/third_party/` - upstream third-party sources such as PX4. These are not
  included in Python package discovery.
- `tests/` - pytest suite covering packaging, config loading, runtime helpers,
  ROS 2 launch assembly, bridge plugins, PX4 scheduling, asset conversion, and
  MuJoCo behavior.

The IDE may show old UE files in open tabs. In the current tree, `acesim/tools/ue5`
is absent and recent history indicates the UE integration was removed from master.
Do not document or edit UE paths unless they reappear in the working tree.

## Setup

Use Python 3.9 or newer.

```bash
pip install -e ".[mujoco]"
```

Other optional dependency groups:

```bash
pip install -e ".[genesis]"
pip install -e ".[all]"
```

The default config uses MuJoCo:

```toml
[basic]
sim_type = "mujoco"
env_type = "am"
scene_name = "default"
asset_name = "x500_arm2x"
benchmark = "multirotor"
```

## Common Commands

Run the local Python entry:

```bash
python -m acesim.core.play
```

Run the ROS 2 launch path:

```bash
ros2 launch acesim_ros2 linux.launch.py
```

Run the headless ROS 2 launch path:

```bash
ros2 launch acesim_ros2 linux_headless.launch.py
```

Regenerate README asset previews:

```bash
python -m acesim.tools.render_readme_assets
```

Run the two-stage asset conversion workflow:

```bash
python -m acesim.tools.sdf2urdf --source px4 --target advanced_plane
python -m acesim.tools.urdf2mjcf --target advanced_plane
```

Run tests:

```bash
pytest
```

Run code-quality hooks:

```bash
pip install pre-commit
pre-commit run --all-files
```

## Development Conventions

- Prefer small, scoped changes that match the existing module boundaries.
- Keep generated or vendored third-party sources out of unrelated edits.
- Do not change `acesim/third_party/` unless the task explicitly involves upstream
  synchronization.
- Treat `acesim/deploy/aircraft/px4_msgs/` as vendored ROS 2 interface material.
  If `.msg` or `.srv` files change, note that ROS 2 workspaces need a clean rebuild.
- The README gallery order is tested. If you edit gallery image references, keep
  the order aligned with `acesim.tools.render_readme_assets.ASSET_ORDER`.
- Use structured XML/TOML/YAML tooling or the existing helper modules for
  structured data changes. Avoid brittle ad hoc string edits where a parser is
  already available.
- Keep `pyproject.toml` and `setup.py` dependency metadata synchronized.
- The project uses Black with a 120-character line length, isort with the Black
  profile, flake8, mypy with missing imports ignored, and standard pre-commit
  hygiene hooks.

## Testing Notes

Useful targeted checks:

```bash
pytest tests/test_packaging_metadata.py
pytest tests/test_render_readme_assets.py
pytest tests/test_config_loader.py
pytest tests/test_bridge_plugin_registry.py tests/test_bridge_runtime.py
pytest tests/test_px4_sdf_asset_pipeline_structure.py tests/test_px4_sdf_asset_pipeline_runtime.py
```

Some tests may require optional MuJoCo or ROS 2 dependencies. If a dependency is
not installed, report the missing environment requirement instead of masking the
failure.

## Documentation Notes

- The root `README.md` follows the Best-README-Template section rhythm while
  keeping ACESim-specific Chinese content.
- The root `LICENSE` and `NOTICE` files define ACESim's Apache-2.0
  licensing boundary. Preserve third-party and vendored license notices.
- When adding new user-facing commands, include the exact install extra needed
  to run them.
