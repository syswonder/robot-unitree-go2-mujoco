# SPDX-License-Identifier: Apache-2.0
"""Compose an unmodified vendor robot with a small procedural test room."""
from pathlib import Path
import xml.etree.ElementTree as ET
import mujoco


def load_scene(assets, room=True, scene="room"):
    """Make an in-memory wrapper; keep vendor files and physical parameters intact."""
    assets = Path(assets).resolve()
    xml = ET.parse(assets / "model/go2.xml").getroot()
    xml.find("compiler").set("meshdir", str(assets / "model/assets"))
    xml.find("option").set("timestep", "0.005")
    world = xml.find("worldbody")
    if scene == "courtyard":
        # Uniform soft daylight: avoid the old bright spotlight at the origin.
        ET.SubElement(world, "light", pos="0 0 8", dir="-.3 .2 -1",
                      directional="true", diffuse=".45 .45 .45",
                      ambient=".08 .08 .08", specular=".03 .03 .03")
    else:
        ET.SubElement(world, "light", pos="0 0 4", dir="0 0 -1", diffuse="0.8 0.8 0.8")
    ET.SubElement(world, "geom", name="floor", type="plane", size="12 12 0.1",
                  rgba=".34 .38 .40 1" if scene == "courtyard" else "0.8 0.85 0.9 1")
    if scene == "courtyard":
        from .courtyard import build
        build(world, xml.find("asset"))
    elif scene != "room":
        raise ValueError(f"Unknown scene: {scene}")
    elif room:
        for name, pos, size in (("front_wall", "5 0 1", ".1 5 1"),
                                ("back_wall", "-5 0 1", ".1 5 1"),
                                ("left_wall", "0 5 1", "5 .1 1"),
                                ("right_wall", "0 -5 1", "5 .1 1"),
                                ("box", "2 2 .4", ".4 .4 .4")):
            ET.SubElement(world, "geom", name=name, type="box", pos=pos, size=size, rgba=".6 .7 .75 1")
    base = world.find("body[@name='base_link']")
    ET.SubElement(base, "camera", name="front_rgbd", pos=".28 0 .09", xyaxes="0 -1 0 0 0 1", fovy="60")
    ET.SubElement(base, "site", name="sim_lidar", pos="0 0 .15", size=".01", rgba="0 0 0 0")
    ET.SubElement(xml, "visual") if xml.find("visual") is None else None
    if scene == 'courtyard':
        ET.SubElement(xml.find('visual'), 'headlight', ambient='.12 .12 .12',
                      diffuse='.15 .15 .15', specular='0 0 0')
    ET.SubElement(xml.find("visual"), "global", offwidth="1920" if scene=='courtyard' else "640",
                  offheight="1080" if scene=='courtyard' else "480")
    return mujoco.MjModel.from_xml_string(ET.tostring(xml, encoding="unicode"))
