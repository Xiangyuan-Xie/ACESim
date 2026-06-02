from __future__ import annotations

import unittest
from pathlib import Path
from unittest.mock import patch


class FakeCurses:
    A_BOLD = 1
    A_DIM = 2
    A_NORMAL = 0
    A_REVERSE = 4
    KEY_DOWN = 258
    KEY_ENTER = 343
    KEY_F10 = 274
    KEY_LEFT = 260
    KEY_RIGHT = 261
    KEY_UP = 259
    KEY_BACKSPACE = 263
    error = RuntimeError

    def __init__(self) -> None:
        self.started_color = False
        self.initialized_pairs: list[tuple[int, int, int]] = []

    def cbreak(self) -> None:
        pass

    def color_pair(self, value: int) -> int:
        return value

    def curs_set(self, value: int) -> None:
        pass

    def has_colors(self) -> bool:
        return True

    def init_pair(self, pair_id: int, foreground: int, background: int) -> None:
        self.initialized_pairs.append((pair_id, foreground, background))

    def noecho(self) -> None:
        pass

    def start_color(self) -> None:
        self.started_color = True

    def use_default_colors(self) -> None:
        pass


class FakeScreen:
    def __init__(self, *, height: int, width: int, keys: list[int | str] | None = None) -> None:
        self.height = height
        self.width = width
        self.background: tuple[str, int] | None = None
        self.keypad_enabled = False
        self.refreshed = False
        self.writes: list[tuple[int, int, str, int]] = []
        self.keys = list(keys or [])
        self.consumed_keys = 0

    def addstr(self, y: int, x: int, text: str, attr: int) -> None:
        if y < 0 or x < 0 or y >= self.height or x >= self.width:
            raise RuntimeError("out of bounds")
        self.writes.append((y, x, text, attr))

    def addnstr(self, y: int, x: int, text: str, max_width: int, attr: int) -> None:
        self.addstr(y, x, text[:max_width], attr)

    def bkgd(self, text: str, attr: int) -> None:
        self.background = (text, attr)

    def clrtoeol(self) -> None:
        pass

    def erase(self) -> None:
        pass

    def getch(self) -> int:
        self.consumed_keys += 1
        key = self.keys.pop(0)
        if isinstance(key, str):
            return ord(key)
        return key

    def getmaxyx(self) -> tuple[int, int]:
        return (self.height, self.width)

    def get_wch(self) -> int | str:
        self.consumed_keys += 1
        return self.keys.pop(0)

    def getstr(self, _y: int, _x: int, max_chars: int) -> bytes:
        raw = "".join(str(key) for key in self.keys)
        self.keys.clear()
        return raw[:max_chars].encode("utf-8")

    def keypad(self, enabled: bool) -> None:
        self.keypad_enabled = enabled

    def move(self, _y: int, _x: int) -> None:
        pass

    def refresh(self) -> None:
        self.refreshed = True


class SDF2URDFTUITests(unittest.TestCase):
    def test_sdf2urdf_prompt_config_reads_bios_form_values(self) -> None:
        from acesim.tools.sdf2urdf.tui import SDF2URDFTUIConfig, prompt_config

        with (
            patch("acesim.tools.sdf2urdf.tui.available_sources", return_value=("px4",)),
            patch("acesim.tools.sdf2urdf.tui.available_targets", return_value=("advanced_plane",)),
            patch(
                "acesim.tools.sdf2urdf.tui.run_bios_form",
                return_value={"source": "px4", "target": "advanced_plane", "cleanup": True},
            ),
        ):
            result = prompt_config()

        self.assertEqual(result, SDF2URDFTUIConfig(source="px4", target="advanced_plane", cleanup=True))

    def test_run_sdf2urdf_pipeline_invokes_stage_functions(self) -> None:
        from acesim.tools.sdf2urdf.tui import SDF2URDFTUIConfig, run_sdf2urdf_pipeline

        config = SDF2URDFTUIConfig(source="px4", target="advanced_plane", cleanup=True)

        with (
            patch("acesim.tools.sdf2urdf.tui.AssetPaths") as paths_cls,
            patch("acesim.tools.sdf2urdf.tui.generate_manual_meshes_from_sdf") as generate,
            patch("acesim.tools.sdf2urdf.tui.sync_manual_urdf_from_sdf") as sync,
            patch("acesim.tools.sdf2urdf.tui.cleanup_manual_meshes_from_sdf") as cleanup,
        ):
            paths = paths_cls.for_target.return_value
            paths.urdf_path = Path("acesim/env/mujoco/asset/advanced_plane/advanced_plane.urdf")
            result = run_sdf2urdf_pipeline(config)

        self.assertEqual(result, paths.urdf_path)
        generate.assert_called_once()
        sync.assert_called_once()
        cleanup.assert_called_once()
        self.assertEqual(generate.call_args.kwargs["source"], "px4")
        self.assertEqual(sync.call_args.kwargs["source"], "px4")

    def test_sdf2urdf_main_dispatches_to_tui(self) -> None:
        from acesim.tools.sdf2urdf.__main__ import main

        with patch("acesim.tools.sdf2urdf.tui.main", return_value=0) as tui_main:
            result = main(["--tui"])

        self.assertEqual(result, 0)
        tui_main.assert_called_once_with()

    def test_sdf2urdf_main_defaults_to_tui_without_arguments(self) -> None:
        from acesim.tools.sdf2urdf.__main__ import main

        with patch("acesim.tools.sdf2urdf.tui.main", return_value=0) as tui_main:
            result = main([])

        self.assertEqual(result, 0)
        tui_main.assert_called_once_with()


class URDF2MJCFTUITests(unittest.TestCase):
    def test_urdf2mjcf_prompt_config_reads_bios_form_values(self) -> None:
        from acesim.tools.urdf2mjcf.tui import URDF2MJCFTUIConfig, prompt_config

        with (
            patch("acesim.tools.urdf2mjcf.tui.available_targets", return_value=("x500_arm2x",)),
            patch("acesim.tools.urdf2mjcf.tui.AssetPaths") as paths_cls,
            patch(
                "acesim.tools.urdf2mjcf.tui.run_bios_form",
                return_value={
                    "target": "x500_arm2x",
                    "floating": True,
                    "decompose": True,
                    "safety_margin": 0.1,
                    "q0": "joint_1=-1.5708",
                    "mujoco_bin": "/opt/mujoco/compile",
                    "overwrite": True,
                },
            ),
        ):
            paths_cls.for_target.return_value.xml_path.exists.return_value = True
            result = prompt_config()

        self.assertEqual(
            result,
            URDF2MJCFTUIConfig(
                target="x500_arm2x",
                floating=True,
                decompose=True,
                safety_margin=0.1,
                q0="joint_1=-1.5708",
                mujoco_bin="/opt/mujoco/compile",
                overwrite=True,
            ),
        )

    def test_available_q0_joints_reads_nonfixed_nonrotor_urdf_joints(self) -> None:
        from acesim.tools.urdf2mjcf.tui import available_q0_joints

        with patch("acesim.tools.urdf2mjcf.tui.AssetPaths") as paths_cls:
            paths = paths_cls.for_target.return_value
            paths.urdf_path = Path("acesim/env/mujoco/asset/x500_arm2x/x500_arm2x.urdf")

            result = available_q0_joints("x500_arm2x")

        self.assertEqual(result, ("joint_1", "joint_2", "joint_3", "joint_4", "joint_5"))

    def test_collect_q0_values_joins_per_joint_values(self) -> None:
        from acesim.tools.urdf2mjcf.tui import collect_q0_values

        result = collect_q0_values(
            {
                "joint_1": "-1.5708",
                "joint_2": "3.1416",
                "joint_3": "0.0",
                "joint_4": "0.0",
                "joint_5": "",
            },
        )

        self.assertEqual(result, "joint_1=-1.5708,joint_2=3.1416,joint_3=0.0,joint_4=0.0")

    def test_q0_editor_reads_joints_for_current_target(self) -> None:
        from acesim.tools.urdf2mjcf.tui import make_q0_editor
        from acesim.tools.utils.tui_models import BIOSField, BIOSFormState

        state = BIOSFormState(
            title="ACESim Setup",
            fields=[
                BIOSField(key="target", label="Target", value="x500", kind="text"),
                BIOSField(key="q0", label="Initial q0", value="", kind="action"),
            ],
        )
        state.values["target"] = "x500_arm2x"
        screen = FakeScreen(height=10, width=48)
        curses = FakeCurses()

        with (
            patch("acesim.tools.urdf2mjcf.tui.available_q0_joints", return_value=("joint_1", "joint_2")) as joints,
            patch(
                "acesim.tools.urdf2mjcf.tui.run_bios_subform",
                return_value={"joint_1": "-1.5708", "joint_2": "3.1416"},
            ) as form,
        ):
            result = make_q0_editor()(state, screen, curses)

        self.assertEqual(result, "joint_1=-1.5708,joint_2=3.1416")
        joints.assert_called_once_with("x500_arm2x")
        labels = [field.label for field in form.call_args.args[1]]
        self.assertEqual(labels, ["joint_1", "joint_2"])

    def test_q0_editor_reuses_current_curses_screen(self) -> None:
        from acesim.tools.urdf2mjcf.tui import make_q0_editor
        from acesim.tools.utils.tui_models import BIOSField, BIOSFormState

        screen = FakeScreen(height=10, width=48)
        curses = FakeCurses()
        state = BIOSFormState(
            title="ACESim Setup",
            fields=[
                BIOSField(key="target", label="Target", value="x500_arm2x", kind="text"),
                BIOSField(key="q0", label="Initial q0", value="", kind="action"),
            ],
        )

        with (
            patch("acesim.tools.urdf2mjcf.tui.available_q0_joints", return_value=("joint_1",)),
            patch("acesim.tools.urdf2mjcf.tui.run_bios_form") as top_level_form,
            patch(
                "acesim.tools.urdf2mjcf.tui.run_bios_subform",
                return_value={"joint_1": "-1.5708"},
            ) as subform,
        ):
            result = make_q0_editor()(state, screen, curses)

        self.assertEqual(result, "joint_1=-1.5708")
        top_level_form.assert_not_called()
        subform.assert_called_once()
        self.assertIs(subform.call_args.args[2], screen)
        self.assertIs(subform.call_args.args[3], curses)

    def test_run_urdf2mjcf_pipeline_invokes_converter_with_tui_options(self) -> None:
        from acesim.tools.urdf2mjcf.tui import URDF2MJCFTUIConfig, run_urdf2mjcf_pipeline

        config = URDF2MJCFTUIConfig(
            target="x500_arm2x",
            floating=True,
            decompose=True,
            safety_margin=0.1,
            q0="joint_1=-1.5708",
            mujoco_bin="/opt/mujoco/compile",
            overwrite=True,
        )

        with patch("acesim.tools.urdf2mjcf.tui.URDF2MJCFConverter") as converter_cls:
            converter = converter_cls.return_value
            converter.xml_path = Path("acesim/env/mujoco/asset/x500_arm2x/x500_arm2x.xml")
            result = run_urdf2mjcf_pipeline(config)

        self.assertEqual(result, converter.xml_path)
        converter_cls.assert_called_once_with(
            target="x500_arm2x",
            floating=True,
            decompose=True,
            safety_margin=0.1,
            q0="joint_1=-1.5708",
            mujoco_bin="/opt/mujoco/compile",
            overwrite=True,
        )
        converter.run.assert_called_once_with()

    def test_urdf2mjcf_main_dispatches_to_tui(self) -> None:
        from acesim.tools.urdf2mjcf.converter import main

        with patch("acesim.tools.urdf2mjcf.tui.main", return_value=0) as tui_main:
            result = main(["--tui"])

        self.assertEqual(result, 0)
        tui_main.assert_called_once_with()

    def test_urdf2mjcf_main_defaults_to_tui_without_arguments(self) -> None:
        from acesim.tools.urdf2mjcf.converter import main

        with patch("acesim.tools.urdf2mjcf.tui.main", return_value=0) as tui_main:
            result = main([])

        self.assertEqual(result, 0)
        tui_main.assert_called_once_with()

    def test_urdf2mjcf_tui_main_runs_pipeline_after_form_returns(self) -> None:
        from acesim.tools.urdf2mjcf import tui

        with (
            patch(
                "acesim.tools.urdf2mjcf.tui.prompt_config",
                return_value=tui.URDF2MJCFTUIConfig(target="x500_arm2x"),
            ),
            patch("acesim.tools.urdf2mjcf.tui.run_urdf2mjcf_pipeline", return_value=Path("asset.xml")) as pipeline,
        ):
            result = tui.main()

        self.assertEqual(result, 0)
        pipeline.assert_called_once_with(tui.URDF2MJCFTUIConfig(target="x500_arm2x"))


class BIOSFormStateTests(unittest.TestCase):
    def test_tui_modules_follow_acelab_style_names(self) -> None:
        from acesim.tools.utils import tui_app, tui_models, tui_runtime

        self.assertIs(tui_app.BIOSField, tui_models.BIOSField)
        self.assertIs(tui_app.render_menu_row, tui_runtime.render_menu_row)
        self.assertFalse((Path(__file__).resolve().parents[1] / "acesim" / "tools" / "bios_tui.py").exists())

    def test_form_state_clamps_selection(self) -> None:
        from acesim.tools.utils.tui_models import BIOSField, BIOSFormState

        state = BIOSFormState(
            title="ACESim Setup",
            fields=[
                BIOSField(key="target", label="Target", value="x500", kind="text"),
                BIOSField(key="floating", label="Floating", value=False, kind="bool"),
            ],
        )

        state.move(-1)

        self.assertEqual(state.selected_index, 0)

    def test_form_state_toggles_boolean_field(self) -> None:
        from acesim.tools.utils.tui_models import BIOSField, BIOSFormState

        state = BIOSFormState(
            title="ACESim Setup",
            fields=[BIOSField(key="floating", label="Floating", value=False, kind="bool")],
        )

        state.toggle_selected()

        self.assertTrue(state.as_dict()["floating"])

    def test_form_state_cycles_choice_field(self) -> None:
        from acesim.tools.utils.tui_models import BIOSField, BIOSFormState

        state = BIOSFormState(
            title="ACESim Setup",
            fields=[
                BIOSField(
                    key="target",
                    label="Target",
                    value="x500",
                    kind="choice",
                    choices=("x500", "x500_arm2x"),
                )
            ],
        )

        state.toggle_selected()

        self.assertEqual(state.as_dict()["target"], "x500_arm2x")

    def test_form_state_converts_float_input(self) -> None:
        from acesim.tools.utils.tui_models import BIOSField, BIOSFormState

        state = BIOSFormState(
            title="ACESim Setup",
            fields=[BIOSField(key="safety_margin", label="Safety Margin", value=0.05, kind="float")],
        )

        state.update_selected("0.1")

        self.assertEqual(state.as_dict()["safety_margin"], 0.1)

    def test_draw_compacts_small_terminal_instead_of_blocking(self) -> None:
        from acesim.tools.utils.tui_app import _draw
        from acesim.tools.utils.tui_models import BIOSField, BIOSFormState

        screen = FakeScreen(height=8, width=32)
        state = BIOSFormState(
            title="ACESim Setup",
            fields=[BIOSField(key="target", label="Target", value="x500", kind="text")],
        )

        _draw(screen, state, FakeCurses())

        rendered = "\n".join(text for _, _, text, _ in screen.writes)
        self.assertNotIn("Terminal too small", rendered)
        self.assertIn("Target", rendered)
        self.assertIn("F10", rendered)

    def test_render_menu_row_fits_narrow_width(self) -> None:
        from acesim.tools.utils.tui_runtime import render_menu_row

        rendered = render_menu_row("target", "x500_arm2x", selected=True, width=24)

        self.assertEqual(len(rendered), 24)
        self.assertTrue(rendered.startswith(">"))
        self.assertIn("target", rendered)

    def test_scroll_offset_keeps_selected_item_visible(self) -> None:
        from acesim.tools.utils.tui_runtime import compute_scroll_offset

        self.assertEqual(compute_scroll_offset(0, 0, 5, 12), 0)
        self.assertEqual(compute_scroll_offset(5, 0, 5, 12), 1)
        self.assertEqual(compute_scroll_offset(11, 1, 5, 12), 7)

    def test_draw_line_avoids_bottom_right_cell(self) -> None:
        from acesim.tools.utils.tui_app import _draw_line

        screen = FakeScreen(height=24, width=80)

        _draw_line(screen, 23, "Enter=Edit  F10=Run  Esc=Quit", 80, 0)

        self.assertEqual(screen.writes[0][0], 23)
        self.assertEqual(len(screen.writes[0][2]), 79)

    def test_run_requires_confirmation_after_f10(self) -> None:
        from acesim.tools.utils.tui_app import _run_curses_form
        from acesim.tools.utils.tui_models import BIOSField, BIOSFormState

        screen = FakeScreen(height=10, width=48, keys=[FakeCurses.KEY_F10, FakeCurses.KEY_F10])
        state = BIOSFormState(
            title="ACESim Setup",
            fields=[BIOSField(key="target", label="Target", value="x500", kind="text")],
        )

        result = _run_curses_form(screen, state, FakeCurses())

        rendered = "\n".join(text for _, _, text, _ in screen.writes)
        self.assertEqual(result, {"target": "x500"})
        self.assertEqual(screen.consumed_keys, 2)
        self.assertIn("CONFIRM", rendered)

    def test_subform_f10_submits_without_confirmation(self) -> None:
        from acesim.tools.utils.tui_app import _run_curses_form
        from acesim.tools.utils.tui_models import BIOSField, BIOSFormState

        screen = FakeScreen(height=10, width=48, keys=[FakeCurses.KEY_F10])
        state = BIOSFormState(
            title="ACESim q0 Setup",
            fields=[BIOSField(key="joint_1", label="joint_1", value="-1.5708", kind="text")],
        )

        result = _run_curses_form(screen, state, FakeCurses(), configure=False, confirm_on_f10=False)

        rendered = "\n".join(text for _, _, text, _ in screen.writes)
        self.assertEqual(result, {"joint_1": "-1.5708"})
        self.assertEqual(screen.consumed_keys, 1)
        self.assertNotIn("CONFIRM", rendered)

    def test_action_field_uses_callback_value(self) -> None:
        from acesim.tools.utils.tui_app import _handle_main_key
        from acesim.tools.utils.tui_models import BIOSField, BIOSFormState

        def edit_q0(state: BIOSFormState, stdscr: FakeScreen, curses_module: FakeCurses) -> str:
            self.assertIs(stdscr, screen)
            self.assertIs(curses_module, curses)
            return f"{state.values['target']}:joint_1=1.0"

        screen = FakeScreen(height=10, width=48)
        curses = FakeCurses()
        state = BIOSFormState(
            title="ACESim Setup",
            fields=[
                BIOSField(key="target", label="Target", value="x500_arm2x", kind="text"),
                BIOSField(key="q0", label="Initial q0", value="", kind="action", editor=edit_q0),
            ],
            selected_index=1,
        )

        _handle_main_key(screen, 10, state, curses)

        self.assertEqual(state.values["q0"], "x500_arm2x:joint_1=1.0")

    def test_main_footer_labels_f10_as_confirm(self) -> None:
        from acesim.tools.utils.tui_app import _draw
        from acesim.tools.utils.tui_models import BIOSField, BIOSFormState

        screen = FakeScreen(height=10, width=48)
        state = BIOSFormState(
            title="ACESim Setup",
            fields=[BIOSField(key="target", label="Target", value="x500", kind="text")],
        )

        _draw(screen, state, FakeCurses())

        rendered = "\n".join(text for _, _, text, _ in screen.writes)
        self.assertIn("F10=Confirm", rendered)

    def test_configure_screen_does_not_initialize_colors(self) -> None:
        from acesim.tools.utils.tui_app import _configure_screen

        curses = FakeCurses()

        _configure_screen(FakeScreen(height=24, width=80), curses)

        self.assertFalse(curses.started_color)
        self.assertEqual(curses.initialized_pairs, [])


if __name__ == "__main__":
    unittest.main()
