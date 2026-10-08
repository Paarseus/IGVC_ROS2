#!/usr/bin/env bash
# One-time Jetson setup for avros_webrtc.
#
#  1. GStreamer runtime pieces not covered by rosdep (rtspclientsink, GI typelibs).
#     nvvidconv / nvv4l2h264enc come with JetPack (nvidia-l4t-gstreamer).
#  2. MediaMTX: official release binary + its shipped default mediamtx.yml,
#     installed UNMODIFIED. Pinned version, checksum verified.
#
# Usage: bash src/avros_webrtc/scripts/install_deps.sh
set -euo pipefail

MTX_VERSION="v1.21.1"
MTX_ARCH="linux_arm64"
MTX_SHA256="6a3aa635fb60ea9b8d566ec306f0a42ff1b6b52a3942bc2baffbe55880d4c3dd"
MTX_TARBALL="mediamtx_${MTX_VERSION}_${MTX_ARCH}.tar.gz"
MTX_URL="https://github.com/bluenviron/mediamtx/releases/download/${MTX_VERSION}/${MTX_TARBALL}"

sudo apt-get update
sudo apt-get install -y \
    python3-gi gir1.2-gstreamer-1.0 gir1.2-gst-plugins-base-1.0 \
    gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad gstreamer1.0-rtsp

# Fail early if the NVIDIA elements are missing (wrong image / not JetPack).
for el in nvvidconv nvv4l2h264enc rtspclientsink h264parse; do
    gst-inspect-1.0 "$el" >/dev/null || { echo "missing GStreamer element: $el" >&2; exit 1; }
done

tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT
curl -fsSL -o "$tmp/$MTX_TARBALL" "$MTX_URL"
echo "${MTX_SHA256}  $tmp/$MTX_TARBALL" | sha256sum -c -
tar -xzf "$tmp/$MTX_TARBALL" -C "$tmp" mediamtx mediamtx.yml

sudo install -m 0755 "$tmp/mediamtx" /usr/local/bin/mediamtx
sudo mkdir -p /usr/local/etc
if [ -e /usr/local/etc/mediamtx.yml ]; then
    echo "/usr/local/etc/mediamtx.yml exists - leaving it alone"
else
    sudo install -m 0644 "$tmp/mediamtx.yml" /usr/local/etc/mediamtx.yml
fi

/usr/local/bin/mediamtx --version
echo "Done. Launch: ros2 launch avros_bringup webrtc.launch.py"
