from __future__ import annotations

import xml.etree.ElementTree as ET

ATTRIB_ORDER = [
    "name",
    "class",
    "type",
    "mesh",
    "material",
    "size",
    "pos",
    "quat",
    "axis",
    "fromto",
    "mass",
    "density",
    "rgba",
    "group",
    "contype",
    "conaffinity",
    "condim",
    "kp",
    "kv",
    "gear",
    "joint",
    "site",
    "objtype",
    "objname",
    "forcerange",
    "ctrlrange",
    "ctrllimited",
    "forcelimited",
]


def indent_xml(elem: ET.Element, level: int = 0) -> None:
    indent = "\n" + level * "  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = indent + "  "
        if not elem.tail or not elem.tail.strip():
            elem.tail = indent
        for child in elem:
            indent_xml(child, level + 1)
        if not child.tail or not child.tail.strip():
            child.tail = indent
        if not elem.tail or not elem.tail.strip():
            elem.tail = indent
    elif level and (not elem.tail or not elem.tail.strip()):
        elem.tail = indent


def sort_attributes(elem: ET.Element) -> None:
    if elem.attrib:
        sorted_attrib: dict[str, str] = {}
        for key in ATTRIB_ORDER:
            if key in elem.attrib:
                sorted_attrib[key] = elem.attrib[key]
        for key, value in elem.attrib.items():
            if key not in sorted_attrib:
                sorted_attrib[key] = value
        elem.attrib.clear()
        elem.attrib.update(sorted_attrib)

    for child in elem:
        sort_attributes(child)
