from __future__ import annotations

from typing import Any, Sequence, cast

from acesim.tools.utils.tui_models import FIELD_ACTION, FIELD_BOOL, FIELD_CHOICE, BIOSField, BIOSFormState
from acesim.tools.utils.tui_runtime import compute_scroll_offset, render_menu_row, truncate_for_width

ESC_KEY = 27


def run_bios_form(title: str, fields: Sequence[BIOSField]) -> dict[str, object] | None:
    try:
        import curses
    except ImportError as exc:  # pragma: no cover - platform guard
        raise RuntimeError("BIOS TUI requires a curses-capable terminal") from exc

    state = BIOSFormState(title=title, fields=fields)
    return curses.wrapper(_run_curses_form, state, curses)


def run_bios_subform(
    title: str,
    fields: Sequence[BIOSField],
    stdscr: Any,
    curses_module: Any,
) -> dict[str, object] | None:
    state = BIOSFormState(title=title, fields=fields)
    return _run_curses_form(stdscr, state, curses_module, configure=False, confirm_on_f10=False)


def _run_curses_form(
    stdscr: Any,
    state: BIOSFormState,
    curses_module: Any,
    *,
    configure: bool = True,
    confirm_on_f10: bool = True,
) -> dict[str, object] | None:
    if configure:
        _configure_screen(stdscr, curses_module)
    while True:
        if state.confirming:
            _draw_confirmation(stdscr, state, curses_module)
        else:
            _draw(stdscr, state, curses_module)

        key = stdscr.getch()
        if state.confirming:
            result = _handle_confirmation_key(key, state, curses_module)
        else:
            result = _handle_main_key(stdscr, key, state, curses_module, confirm_on_f10=confirm_on_f10)
        if result is not _NO_RESULT:
            if result is None:
                return None
            return cast(dict[str, object], result)


def _configure_screen(stdscr: Any, curses_module: Any) -> None:
    curses_module.curs_set(0)
    curses_module.noecho()
    curses_module.cbreak()
    stdscr.keypad(True)


def _handle_main_key(
    stdscr: Any,
    key: int,
    state: BIOSFormState,
    curses_module: Any,
    *,
    confirm_on_f10: bool = True,
) -> object:
    if key in (curses_module.KEY_UP, ord("k")):
        state.move(-1)
    elif key in (curses_module.KEY_DOWN, ord("j")):
        state.move(1)
    elif key in (curses_module.KEY_LEFT, ord("h")):
        state.cycle_selected(-1)
    elif key in (curses_module.KEY_RIGHT, ord("l")):
        state.cycle_selected(1)
    elif key == ord(" "):
        state.toggle_selected()
    elif key in _enter_keys(curses_module):
        _activate_selected(stdscr, state, curses_module)
    elif key == curses_module.KEY_F10:
        if not confirm_on_f10:
            return state.as_dict()
        state.confirming = True
    elif key in (ESC_KEY, ord("q"), ord("Q")):
        return None
    return _NO_RESULT


def _handle_confirmation_key(key: int, state: BIOSFormState, curses_module: Any) -> object:
    if key in _enter_keys(curses_module) or key == curses_module.KEY_F10:
        return state.as_dict()
    if key in (ESC_KEY, ord("q"), ord("Q")):
        state.confirming = False
        state.status = "Returned to setup."
    return _NO_RESULT


def _activate_selected(stdscr: Any, state: BIOSFormState, curses_module: Any) -> None:
    field = state.selected_field
    if field.kind == FIELD_BOOL:
        state.toggle_selected()
        state.status = f"Updated {field.label}."
        return
    if field.kind == FIELD_ACTION and field.editor is not None:
        value = field.editor(state, stdscr, curses_module)
        if value is None:
            state.status = f"Cancelled edit for {field.label}."
            return
        state.values[field.key] = value
        state.status = f"Updated {field.label}."
        return
    if field.kind == FIELD_CHOICE and field.choices:
        selected_value = _choose_option(stdscr, state, field, curses_module)
        if selected_value is None:
            state.status = f"Cancelled edit for {field.label}."
            return
        state.values[field.key] = selected_value
        state.status = f"Updated {field.label}."
        return

    value = _read_value(stdscr, state, curses_module)
    if value is None:
        state.status = f"Cancelled edit for {field.label}."
        return
    try:
        state.update_selected(value)
        state.status = f"Updated {field.label}."
    except ValueError as exc:
        state.status = str(exc)


def _choose_option(stdscr: Any, state: BIOSFormState, field: BIOSField, curses_module: Any) -> str | None:
    current_value = str(state.values.get(field.key, ""))
    current_index = 0
    for index, value in enumerate(field.choices):
        if value == current_value:
            current_index = index
            break
    scroll_offset = 0

    while True:
        stdscr.erase()
        height, width = stdscr.getmaxyx()
        viewport_height = max(height - 4, 1)
        scroll_offset = compute_scroll_offset(current_index, scroll_offset, viewport_height, len(field.choices))
        _draw_line(stdscr, 0, f" SELECT {field.label} ", width, _attr(curses_module, "A_BOLD"))
        _draw_line(stdscr, 1, "=" * width, width, _attr(curses_module, "A_DIM"))
        visible_options = tuple(field.choices)[scroll_offset : scroll_offset + viewport_height]
        for row_offset, option in enumerate(visible_options):
            option_index = scroll_offset + row_offset
            row_text = truncate_for_width(f"{'> ' if option_index == current_index else '  '}{option}", width)
            row_attr = (
                _attr(curses_module, "A_REVERSE") if option_index == current_index else _attr(curses_module, "A_NORMAL")
            )
            _draw_line(stdscr, 2 + row_offset, row_text, width, row_attr)
        _draw_line(stdscr, height - 1, "Enter=Select  Esc=Back", width, _attr(curses_module, "A_BOLD"))
        stdscr.refresh()

        key = stdscr.getch()
        if key in (curses_module.KEY_UP, ord("k")):
            current_index = max(current_index - 1, 0)
        elif key in (curses_module.KEY_DOWN, ord("j")):
            current_index = min(current_index + 1, len(field.choices) - 1)
        elif key in _enter_keys(curses_module):
            return str(field.choices[current_index])
        elif key in (ESC_KEY, ord("q"), ord("Q")):
            return None


def _read_value(stdscr: Any, state: BIOSFormState, curses_module: Any) -> str | None:
    field = state.selected_field
    current_text = str(state.values.get(field.key, ""))
    entered_chars: list[str] = []

    curses_module.noecho()
    curses_module.curs_set(1)
    try:
        while True:
            height, width = stdscr.getmaxyx()
            prompt_y = max(height - 3, 0)
            input_y = max(height - 2, 0)
            prompt = truncate_for_width(
                f"Set {field.label} (blank keeps current). Current: {current_text or '<auto>'}", width
            )
            entered_text = "".join(entered_chars)
            input_width = max(width - 2, 1)
            visible_text = entered_text[-input_width:]

            _draw_line(stdscr, prompt_y, prompt, width, _attr(curses_module, "A_DIM"))
            _draw_line(stdscr, input_y, f"> {visible_text}", width, _attr(curses_module, "A_NORMAL"))
            if hasattr(stdscr, "move") and width > 2:
                stdscr.move(input_y, min(2 + len(visible_text), width - 1))
            stdscr.refresh()

            key = stdscr.get_wch() if hasattr(stdscr, "get_wch") else chr(stdscr.getch())
            if key in _enter_keys(curses_module) or key in ("\n", "\r"):
                value = entered_text.strip()
                return value if value else None
            if key in (ESC_KEY, "\x1b"):
                return None
            if key in (getattr(curses_module, "KEY_BACKSPACE", 263), 127, "\b", "\x7f"):
                if entered_chars:
                    entered_chars.pop()
                continue
            if isinstance(key, str) and key.isprintable():
                entered_chars.append(key)
    finally:
        curses_module.noecho()
        curses_module.curs_set(0)


def _draw(stdscr: Any, state: BIOSFormState, curses_module: Any) -> None:
    stdscr.erase()
    height, width = stdscr.getmaxyx()
    if height <= 0 or width <= 0:
        return

    detail_y = max(height - 3, 0)
    footer_y = max(height - 1, 0)
    viewport_top = 2
    viewport_height = max(detail_y - viewport_top, 1)
    state.scroll_offset = compute_scroll_offset(
        state.selected_index,
        state.scroll_offset,
        viewport_height,
        len(state.fields),
    )

    _draw_line(stdscr, 0, f" {state.title} ", width, _attr(curses_module, "A_BOLD"))
    _draw_line(stdscr, 1, "=" * width, width, _attr(curses_module, "A_DIM"))

    visible_fields = state.fields[state.scroll_offset : state.scroll_offset + viewport_height]
    for row_offset, item in enumerate(visible_fields):
        index = state.scroll_offset + row_offset
        row_text = render_menu_row(item.label, state.formatted_value(item), index == state.selected_index, width)
        row_attr = (
            _attr(curses_module, "A_REVERSE") if index == state.selected_index else _attr(curses_module, "A_NORMAL")
        )
        _draw_line(stdscr, viewport_top + row_offset, row_text, width, row_attr)

    detail_text = _build_detail_text(state.selected_field)
    _draw_line(stdscr, detail_y, detail_text, width, _attr(curses_module, "A_DIM"))
    if height >= 2:
        _draw_line(stdscr, min(detail_y + 1, footer_y), state.status, width, _attr(curses_module, "A_NORMAL"))
    _draw_line(stdscr, footer_y, "Enter=Edit  F10=Confirm  Esc=Quit", width, _attr(curses_module, "A_BOLD"))
    stdscr.refresh()


def _draw_confirmation(stdscr: Any, state: BIOSFormState, curses_module: Any) -> None:
    stdscr.erase()
    height, width = stdscr.getmaxyx()
    if height <= 0 or width <= 0:
        return

    _draw_line(stdscr, 0, f" CONFIRM {state.title} ", width, _attr(curses_module, "A_BOLD"))
    _draw_line(stdscr, 1, "=" * width, width, _attr(curses_module, "A_DIM"))
    _draw_line(stdscr, 2, "Final configuration:", width, _attr(curses_module, "A_BOLD"))

    max_rows = max(height - 5, 1)
    for row, item in enumerate(state.fields[:max_rows]):
        value = state.formatted_value(item)
        _draw_line(stdscr, 3 + row, render_menu_row(item.label, value, False, width), width)

    _draw_line(stdscr, height - 1, "Enter=Launch  F10=Launch  Esc=Back", width, _attr(curses_module, "A_BOLD"))
    stdscr.refresh()


def _build_detail_text(item: BIOSField) -> str:
    help_text = item.help or "No help text."
    return f"{item.label} | {help_text}"


def _draw_line(stdscr: Any, y: int, text: str, width: int, attr: int = 0) -> None:
    if y < 0:
        return
    height, actual_width = stdscr.getmaxyx()
    if y >= height or width <= 0 or actual_width <= 0:
        return

    draw_width = min(width, actual_width)
    if y == height - 1 and draw_width > 1:
        draw_width -= 1
    rendered = truncate_for_width(text, draw_width).ljust(draw_width)
    if hasattr(stdscr, "addnstr"):
        stdscr.addnstr(y, 0, rendered, draw_width, attr)
    else:
        stdscr.addstr(y, 0, rendered, attr)


def _enter_keys(curses_module: Any) -> set[int]:
    return {10, 13, getattr(curses_module, "KEY_ENTER", 343)}


def _attr(curses_module: Any, name: str) -> int:
    return int(getattr(curses_module, name, 0))


_NO_RESULT = object()
