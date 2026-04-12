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
    i = "\n" + level * "  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + "  "
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
        for child in elem:
            indent_xml(child, level + 1)
        if not child.tail or not child.tail.strip():
            child.tail = i
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
    elif level and (not elem.tail or not elem.tail.strip()):
        elem.tail = i


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
