import os
from xml.dom import minidom

import xacro


def urdf_to_oneline_without_comments(xml_string: str) -> str:
    dom = minidom.parseString(xml_string)

    def remove_comments(node):
        for child in list(node.childNodes):
            if child.nodeType == child.COMMENT_NODE:
                node.removeChild(child)
            else:
                remove_comments(child)

    remove_comments(dom)
    return dom.toxml()


def xacro_to_urdf_xml(xacro_path: str, mappings: dict[str, str]) -> str:
    doc = xacro.parse(open(xacro_path))
    xacro.process_doc(doc, mappings=mappings)
    return doc.toxml()


def build_kuroko_urdf_for_spawn(xacro_path: str, controller_yaml: str, gazebo: bool) -> str:
    xml_raw = xacro_to_urdf_xml(
        xacro_path,
        {
            "gazebo": "true" if gazebo else "false",
            "controller_yaml": controller_yaml,
        },
    )
    return urdf_to_oneline_without_comments(xml_raw)
