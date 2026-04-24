"""Class-map loader.

Parses class_map.yaml and builds a vision_msgs/LabelInfo message the node
can latch. Keeping this in one module means pipelines, config, and the
kiwicampus costmap plugin share a single source of truth.
"""

from dataclasses import dataclass

import yaml

from vision_msgs.msg import LabelInfo, VisionClass


@dataclass
class ClassEntry:
    id: int
    name: str
    rgb: tuple  # (r, g, b), 0..255


def load_class_map(path: str) -> list[ClassEntry]:
    with open(path, 'r') as f:
        data = yaml.safe_load(f)
    entries = []
    for c in data.get('classes', []):
        entries.append(ClassEntry(
            id=int(c['id']),
            name=str(c['name']),
            rgb=tuple(int(v) for v in c.get('rgb', (128, 128, 128))),
        ))
    return entries


def build_label_info(entries: list[ClassEntry], frame_id: str = '') -> LabelInfo:
    msg = LabelInfo()
    msg.header.frame_id = frame_id
    for e in entries:
        vc = VisionClass()
        vc.class_id = e.id
        vc.class_name = e.name
        msg.class_map.append(vc)
    return msg
