from __future__ import annotations


def truncate_for_width(text: str, width: int) -> str:
    if width <= 0:
        return ""
    if len(text) <= width:
        return text
    if width <= 3:
        return "." * width
    return text[: width - 3] + "..."


def compute_scroll_offset(selected_index: int, current_offset: int, viewport_height: int, total_items: int) -> int:
    if total_items <= 0 or viewport_height <= 0:
        return 0

    max_offset = max(total_items - viewport_height, 0)
    offset = min(max(current_offset, 0), max_offset)
    if selected_index < offset:
        return selected_index
    if selected_index >= offset + viewport_height:
        return min(selected_index - viewport_height + 1, max_offset)
    return offset


def render_menu_row(label: str, value: str, selected: bool, width: int) -> str:
    if width <= 0:
        return ""

    prefix = "> " if selected else "  "
    if width <= len(prefix):
        return truncate_for_width(prefix, width).ljust(width)

    available_width = width - len(prefix)
    separator = " : "
    minimum_value_width = 4
    max_label_width = max(available_width - len(separator) - minimum_value_width, 1)
    label_width = min(max(len(label), 8), max_label_width, 24)
    if label_width + len(separator) + minimum_value_width > available_width:
        label_width = max(available_width - len(separator) - minimum_value_width, 1)

    label_text = truncate_for_width(label, label_width).ljust(label_width)
    value_width = max(available_width - label_width - len(separator), 0)
    value_text = truncate_for_width(value, value_width)
    return truncate_for_width(f"{prefix}{label_text}{separator}{value_text}", width).ljust(width)
