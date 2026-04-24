"""
Tests for the class-map YAML loader + LabelInfo builder.

Pins the single-source-of-truth contract: class_map.yaml parses into
ClassEntry records that serialize cleanly into vision_msgs/LabelInfo
for the kiwicampus plugin to consume. Catches silent drift between
YAML schema changes and the LabelInfo message shape.
"""

import os

import pytest
import yaml

from avros_perception.utils.class_map import (
    ClassEntry,
    build_label_info,
    load_class_map,
)


SHIPPED_CLASS_MAP = os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..', '..', 'config', 'class_map.yaml'
))


@pytest.fixture
def tmp_yaml(tmp_path):
    """Return a factory that writes a classes-only YAML under tmp_path."""
    def _make(classes):
        path = tmp_path / 'class_map.yaml'
        path.write_text(yaml.dump({'classes': classes}))
        return str(path)
    return _make


def test_shipped_class_map_parses():
    """Real class_map.yaml in share/ must load without error."""
    entries = load_class_map(SHIPPED_CLASS_MAP)
    assert len(entries) >= 1
    # Every entry is well-formed
    for e in entries:
        assert isinstance(e, ClassEntry)
        assert isinstance(e.id, int)
        assert isinstance(e.name, str) and e.name
        assert len(e.rgb) == 3


def test_shipped_class_map_has_free_and_unknown():
    """Stub pipeline writes 0 (free) and 255 (unknown) — must be in the map."""
    entries = load_class_map(SHIPPED_CLASS_MAP)
    by_id = {e.id: e.name for e in entries}
    assert 0 in by_id, 'class_map.yaml missing class 0 (free)'
    assert 255 in by_id, 'class_map.yaml missing class 255 (unknown)'


def test_load_preserves_id_name_rgb(tmp_yaml):
    path = tmp_yaml([
        {'id': 7, 'name': 'foo', 'rgb': [10, 20, 30]},
    ])
    [entry] = load_class_map(path)
    assert entry.id == 7
    assert entry.name == 'foo'
    assert entry.rgb == (10, 20, 30)


def test_load_defaults_rgb_when_missing(tmp_yaml):
    path = tmp_yaml([{'id': 1, 'name': 'bar'}])
    [entry] = load_class_map(path)
    assert entry.rgb == (128, 128, 128)


def test_load_empty_class_list(tmp_yaml):
    path = tmp_yaml([])
    assert load_class_map(path) == []


def test_load_missing_classes_key_is_empty(tmp_path):
    path = tmp_path / 'no_classes.yaml'
    path.write_text('unrelated_root: 1\n')
    assert load_class_map(str(path)) == []


def test_build_label_info_round_trip():
    entries = [
        ClassEntry(id=0, name='free', rgb=(0, 0, 0)),
        ClassEntry(id=5, name='danger', rgb=(0, 0, 0)),
        ClassEntry(id=255, name='unknown', rgb=(0, 0, 0)),
    ]
    info = build_label_info(entries)
    assert len(info.class_map) == 3
    ids = [v.class_id for v in info.class_map]
    names = [v.class_name for v in info.class_map]
    assert ids == [0, 5, 255]
    assert names == ['free', 'danger', 'unknown']


def test_build_label_info_frame_id():
    info = build_label_info([], frame_id='zed_front_camera_link')
    assert info.header.frame_id == 'zed_front_camera_link'


def test_build_label_info_empty_entries():
    info = build_label_info([])
    assert len(info.class_map) == 0
    assert info.header.frame_id == ''
