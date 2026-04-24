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
EXPECTED_HASH = '4aeb9acc7fb1b24611138db5956b1c99d711018a3ac3f578ed61754150a4990a'


def _hash_hsv_params():
    with open(PERCEPTION_YAML) as f:
        data = yaml.safe_load(f)
    params = data['perception_node']['ros__parameters']
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
    with open(PERCEPTION_YAML) as f:
        params = yaml.safe_load(f)['perception_node']['ros__parameters']
    missing = [k for k in _HSV_PARAM_KEYS if k not in params]
    assert not missing, f'perception.yaml missing: {missing}'
