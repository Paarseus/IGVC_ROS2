# Xsens MTi-680G: check and restore the device configuration

Run these on a laptop with the Xsens MT Software Suite installed, with the MTi on USB (`/dev/ttyUSB0`). They use the Xsens Python SDK (`xsensdeviceapi`). The SDK wheels only work with Python ≤ 3.11, and the cp311 wheel is broken (it links `libpython3.9`), so use a Python 3.9 environment:

```bash
uv venv -p 3.9 ~/xda39 && source ~/xda39/bin/activate
uv pip install /usr/local/xsens/python/xsensdeviceapi-2025.2.0-cp39-none-linux_x86_64.whl
export LD_LIBRARY_PATH=$(python -c "import sysconfig;print(sysconfig.get_config_var('LIBDIR'))")
```

| Script | What it does | Writes to the device? |
|---|---|---|
| `xsens_inspect.py [port] [115200|921600]` | prints product, firmware, filter profile, u-blox platform, GNSS lever arm, receiver settings, option flags, baud, output configuration | no |
| `xsens_live.py [seconds]` | live GNSS status from the receiver: fix type, satellites, RTK carrier solution, accuracy | no |
| `xsens_restore.py` | writes the reference configuration below (opens at 115200), sets 921600 baud, resets the device | **yes** |

## Reference configuration (verified 2026-09-30, firmware 1.16.0 build 260820008)
| Setting | Value |
|---|---|
| Filter profile | General_RTK |
| u-blox platform | 4 (automotive) |
| GNSS lever arm | [0.74, 0, 0] m |
| GNSS receiver settings | [3, 1, 4, 4] (3 = ZED-F9P, 4 Hz) |
| Option flags | 0x1A20 = orientation smoother, position/velocity smoother, continuous ZRU, config message at start-up |
| Serial baud | 921600 |
| Outputs | 0x1020 packet counter, 0x1060 sample time fine, 0x1010 UTC time, 0xE020 status word (all with every packet); 0x4020 acceleration, 0x8020 rate of turn, 0xC020 magnetic field, 0x2010 quaternion, 0x4030 free acceleration, 0x5042 lat/lon, 0x5022 altitude, 0xD012 velocity (100 Hz); 0x7010 GNSS PVT data (4 Hz) |

## After a firmware update
The Xsens Firmware Updater (seen with 4.5.1, 1.12.0 → 1.16.0) resets the baud to 115200 and most settings to factory defaults. It keeps the filter profile. Steps:
1. `xsens_inspect.py /dev/ttyUSB0 115200` to see what changed.
2. `xsens_restore.py`.
3. `xsens_inspect.py /dev/ttyUSB0 921600`: every row must match the table above.
