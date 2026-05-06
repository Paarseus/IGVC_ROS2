#!/usr/bin/env bash
# Install systemd units in this directory under /etc/systemd/system,
# reload, then enable + start each one. Run on the Jetson.
set -euo pipefail

DIR="$(cd "$(dirname "$0")" && pwd)"

units=()
for f in "$DIR"/*.service; do
  [ -e "$f" ] || continue
  units+=("$(basename "$f")")
  sudo install -m 644 "$f" "/etc/systemd/system/$(basename "$f")"
done

if [ ${#units[@]} -eq 0 ]; then
  echo "no .service files found in $DIR"
  exit 1
fi

sudo systemctl daemon-reload
for u in "${units[@]}"; do
  sudo systemctl enable --now "$u"
  sudo systemctl status "$u" --no-pager -l | head -15
done
