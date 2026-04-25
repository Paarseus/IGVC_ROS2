"""
HSV-threshold immutability guard.

Freezes the SHA256 of the HSV-tuning fields in perception.yaml. Any PR
that touches a threshold must also update EXPECTED_HASH here — which
forces the change to surface explicitly in review (no silent drift).

When you intentionally retune thresholds:
  1. Update perception.yaml.
  2. Run this test; it prints the new hash in the failure message.
  3. Paste the new hash into EXPECTED_HASH and commit together.
"""

import hashlib
import os

import yaml


PERCEPTION_YAML = os.path.abspath(os.path.join(
    os.path.dirname(__file__), '..', '..', 'config', 'perception.yaml'
))

# HSV tuning fields whose drift must be reviewed.
_HSV_PARAM_KEYS = [
    'blur_iters', 'adaptive_period', 'adaptive_k',
    'lane_low', 'lane_high', 'class_id_lane',
    'barrel_low', 'barrel_high', 'class_id_barrel',
    'pothole_low', 'pothole_high', 'class_id_pothole',
    'sky_roi_poly',
]

# Frozen hash. Update when retuning intentionally.
EXPECTED_HASH = 'aca1c4d16f5fddef0fad13db5e9b4cfe58b4ca8daa9732bbb1086ee507e42437'


def _load_params():
    """Read perception.yaml's ros__parameters, tolerant of root key name."""
    with open(PERCEPTION_YAML) as f:
        data = yaml.safe_load(f)
    # Accept either explicit 'perception_node' or wildcard '/**' root key.
    for root_key in ('/**', 'perception_node'):
        if root_key in data and 'ros__parameters' in data[root_key]:
            return data[root_key]['ros__parameters']
    raise KeyError(
        f'{PERCEPTION_YAML}: no ros__parameters under /** or perception_node'
    )


def _hash_hsv_params():
    params = _load_params()
    extracted = {k: params.get(k) for k in _HSV_PARAM_KEYS}
    serialized = yaml.dump(extracted, sort_keys=True).encode('utf-8')
    return hashlib.sha256(serialized).hexdigest()


def test_hsv_thresholds_unchanged():
    """
    Assert perception.yaml's HSV-tuning fields match the frozen hash.

    A failure here means someone touched an HSV threshold. If that was
    intentional, update EXPECTED_HASH to the value printed in this
    assertion message.
    """
    actual = _hash_hsv_params()
    assert actual == EXPECTED_HASH, (
        f'HSV params changed.\n'
        f'  expected: {EXPECTED_HASH}\n'
        f'  actual:   {actual}\n'
        f'If intentional, update EXPECTED_HASH in this file.'
    )


def test_hsv_param_keys_all_present():
    """perception.yaml must declare every key the hash tracks."""
    params = _load_params()
    missing = [k for k in _HSV_PARAM_KEYS if k not in params]
    assert not missing, f'perception.yaml missing: {missing}'
