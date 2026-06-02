from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Callable, Sequence

FIELD_ACTION = "action"
FIELD_BOOL = "bool"
FIELD_CHOICE = "choice"
FIELD_FLOAT = "float"
FIELD_TEXT = "text"


@dataclass(frozen=True)
class BIOSField:
    key: str
    label: str
    value: object = ""
    kind: str = FIELD_TEXT
    choices: Sequence[str] = ()
    help: str = ""
    editor: Callable[["BIOSFormState", Any, Any], object | None] | None = None

    def __post_init__(self) -> None:
        object.__setattr__(self, "choices", tuple(self.choices))


@dataclass
class BIOSFormState:
    title: str
    fields: Sequence[BIOSField]
    selected_index: int = 0
    values: dict[str, object] = field(default_factory=dict)
    status: str = ""
    scroll_offset: int = 0
    confirming: bool = False

    def __post_init__(self) -> None:
        self.fields = tuple(self.fields)
        if not self.fields:
            raise ValueError("BIOS form requires at least one field")
        if not self.values:
            self.values = {item.key: item.value for item in self.fields}
        self.selected_index = min(max(self.selected_index, 0), len(self.fields) - 1)
        if not self.status:
            self.status = "Use Up/Down to move. Enter edits. F10 confirms. Esc quits."

    @property
    def selected_field(self) -> BIOSField:
        return self.fields[self.selected_index]

    def move(self, delta: int) -> None:
        self.selected_index = min(max(self.selected_index + delta, 0), len(self.fields) - 1)

    def toggle_selected(self) -> None:
        field = self.selected_field
        if field.kind == FIELD_BOOL:
            self.values[field.key] = not bool(self.values[field.key])
            return
        if field.kind == FIELD_CHOICE:
            self.cycle_selected(1)

    def cycle_selected(self, delta: int) -> None:
        field = self.selected_field
        if field.kind != FIELD_CHOICE or not field.choices:
            return
        current = str(self.values.get(field.key, field.value))
        try:
            index = field.choices.index(current)
        except ValueError:
            index = 0
        self.values[field.key] = field.choices[(index + delta) % len(field.choices)]

    def update_selected(self, raw_value: str) -> None:
        field = self.selected_field
        value = raw_value.strip()
        if not value:
            return
        if field.kind == FIELD_BOOL:
            self.values[field.key] = value.lower() in {"1", "on", "true", "y", "yes"}
        elif field.kind == FIELD_FLOAT:
            self.values[field.key] = float(value)
        elif field.kind == FIELD_CHOICE:
            if field.choices and value not in field.choices:
                raise ValueError(f"{value} is not a valid value for {field.label}")
            self.values[field.key] = value
        else:
            self.values[field.key] = value

    def formatted_value(self, field: BIOSField) -> str:
        value = self.values.get(field.key, field.value)
        if field.kind == FIELD_BOOL:
            return "Enabled" if bool(value) else "Disabled"
        if field.kind == FIELD_ACTION:
            return str(value) if value else "<open>"
        if field.kind == FIELD_FLOAT:
            return f"{float(str(value)):g}"
        if value == "":
            return "<auto>"
        return str(value)

    def as_dict(self) -> dict[str, object]:
        return dict(self.values)
