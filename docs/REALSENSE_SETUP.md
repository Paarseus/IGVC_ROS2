# RealSense D455 Setup Guide (Jetson Orin, JetPack 6)

This documents the full procedure to get the RealSense D455 working on the Jetson Orin running JetPack 6 (R36.x, kernel 5.15-tegra). The standard apt packages do not work due to HID/V4L2 incompatibilities.

## Problem

The D455 crashes with `std::bad_optional_access` when launched via the ROS2 node. Root cause: firmware 5.16+ presents HID descriptors that the RSUSB userspace backend cannot handle on JetPack 6. The `ds-motion-common.cpp` code logs "No HID info provided, IMU is disabled" then a `std::optional::value()` call on an empty optional throws, crashing the entire device initialization.

References:
- [librealsense #14169](https://github.com/IntelRealSense/librealsense/issues/14169) — D455 + RSUSB + FW 5.16+
- [librealsense #13341](https://github.com/IntelRealSense/librealsense/issues/13341) — JetPack 6 removed HID/hidraw
- [realsense-ros #3416](https://github.com/realsenseai/realsense-ros/issues/3416) — exact crash report

## Step 1: Build librealsense 2.57.6 from source with RSUSB

```bash
cd ~/librealsense
mkdir -p build && cd build
cmake .. \
  -DFORCE_RSUSB_BACKEND=ON \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_EXAMPLES=true \
  -DBUILD_WITH_CUDA=true
make -j6    # -j6 not -j8 to avoid OOM on Orin
sudo make install
sudo ldconfig
```

Installs to `/usr/local/lib/` and `/usr/local/include/`.

## Step 2: Downgrade D455 firmware to 5.13.0.50

```bash
# Download firmware
wget https://librealsense.intel.com/Releases/RS4xx/FW/D4XX_FW_Image-5.13.0.50.bin -O ~/D4XX_FW_Image-5.13.0.50.bin

# Flash (camera must be connected via USB)
rs-fw-update -f ~/D4XX_FW_Image-5.13.0.50.bin

# Verify
rs-enumerate-devices --compact
# Should show: Intel RealSense D455  5.13.0.50  USB 3.2
```

## Step 3: Remove apt librealsense (prevents header/library conflicts)

The apt package `ros-humble-librealsense2` installs v2.56.4 headers at `/opt/ros/humble/include/librealsense2/` which shadow the correct v2.57.6 headers at `/usr/local/include/librealsense2/`. CMake's ament include path ordering puts apt headers first, causing realsense-ros to compile with stale version strings even when `-Drealsense2_DIR` points to the local build.

```bash
sudo apt remove ros-humble-librealsense2 ros-humble-librealsense2-dbgsym
# This also removes ros-humble-realsense2-camera (apt version) — we use source build anyway
```

After removal, only `/usr/local/` provides librealsense2 headers and libraries.

## Step 4: Build realsense-ros 4.56.4 from source

```bash
# Clone via vcstool (preferred) or manually:
cd ~/IGVC
vcs import src < avros.repos   # clones realsense-ros 4.56.4 + xsens_mti

colcon build --symlink-install \
  --packages-select realsense2_camera_msgs realsense2_description realsense2_camera \
  --cmake-args -Drealsense2_DIR=/usr/local/lib/cmake/realsense2
source install/setup.bash
```

**Why 4.56.4 and not 4.57.6?** Tag 4.57.6 adds D457 safety camera features (`RS2_STREAM_SAFETY`, `RS2_STREAM_LABELED_POINT_CLOUD`, `RS2_STREAM_OCCUPANCY`) that don't exist in librealsense 2.57.6 — the build fails with undeclared identifier errors. Tag 4.56.4 requires `find_package(realsense2 2.56)` which is satisfied by 2.57.6.

## Step 5: Verify

```bash
source /opt/ros/humble/setup.bash
source ~/IGVC/install/setup.bash

ros2 launch realsense2_camera rs_launch.py \
  camera_name:=camera enable_color:=true enable_depth:=true \
  enable_gyro:=false enable_accel:=false

# Expected output:
#   Built with LibRealSense v2.57.6
#   Running with LibRealSense v2.57.6
#   Device Name: Intel RealSense D455
#   Device FW version: 5.13.0.50
#   RealSense Node Is Up!
```

## Pitfalls encountered

1. **Stale build artifacts** — After checking out a new librealsense version, `make` can use old `.o` files. Always `rm -rf build/` or `make clean` before rebuilding.
2. **CMake finding apt before local** — Even with `-Drealsense2_DIR=/usr/local/lib/cmake/realsense2`, the ament build system adds `-isystem /opt/ros/humble/include` before the local include path. The compiler finds apt headers first. Removing the apt package is the only reliable fix.
3. **`--allow-overriding` flag** — Colcon on Humble doesn't support this flag. Not needed if the apt realsense2_camera package is removed.
4. **Symlink conflicts in install/** — If rebuilding after a failed build, stale symlinks can cause "File exists" errors. Fix: `rm -rf build/<pkg> install/<pkg>` before rebuilding.
5. **USB interface busy** — A previous camera node holds the USB interface. Always `pkill -f realsense2_camera_node && sleep 2` before relaunching.
