import xml.etree.ElementTree as ET

from acesim.tools.utils.xml_formatting import ATTRIB_ORDER, indent_xml, sort_attributes

__all__ = [
    "ATTRIB_ORDER",
    "add_collision_exclusions",
    "indent_xml",
    "inject_xml",
    "sort_attributes",
]


def inject_xml(parent: ET.Element, xml_content: str, index: int = -1, *, source: str = "xml fragment") -> None:
    try:
        fragment = ET.fromstring(f"<root>{xml_content}</root>")
    except ET.ParseError as exc:
        raise ValueError(f"Failed to parse {source}: {exc}") from exc
    for child in list(fragment):
        if index < 0:
            parent.append(child)
        else:
            parent.insert(index, child)
            index += 1


def add_collision_exclusions(root: ET.Element) -> None:
    contact = root.find("contact")
    if contact is None:
        contact = ET.Element("contact")
        root.append(contact)

    existing_excludes = set()
    for exclude in contact.findall("exclude"):
        b1 = exclude.get("body1")
        b2 = exclude.get("body2")
        if b1 and b2:
            existing_excludes.add(tuple(sorted((b1, b2))))

    def traverse(body: ET.Element) -> None:
        parent_name = body.get("name")
        children = [child for child in body if child.tag == "body"]

        for i, child in enumerate(children):
            child_name = child.get("name")
            if not parent_name or not child_name:
                continue

            pair = tuple(sorted((parent_name, child_name)))
            if pair not in existing_excludes and parent_name != "world":
                exclude_elem = ET.SubElement(contact, "exclude")
                exclude_elem.set("body1", parent_name)
                exclude_elem.set("body2", child_name)
                existing_excludes.add(pair)

            for sibling in children[:i]:
                sibling_name = sibling.get("name")
                if not sibling_name:
                    continue
                pair_sib = tuple(sorted((child_name, sibling_name)))
                if pair_sib in existing_excludes:
                    continue
                exclude_elem = ET.SubElement(contact, "exclude")
                exclude_elem.set("body1", child_name)
                exclude_elem.set("body2", sibling_name)
                existing_excludes.add(pair_sib)

            traverse(child)

    worldbody = root.find("worldbody")
    if worldbody is not None:
        for child in worldbody.findall("body"):
            traverse(child)
