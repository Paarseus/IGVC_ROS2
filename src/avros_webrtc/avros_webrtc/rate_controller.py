"""AVROS rate controller: shared bitrate scale for all webrtc_node encoders.

Fixed CBR has no congestion control. On cellular the link doesn't drop
when overloaded, it bufferbloats: the modem queue grows and drive
commands arrive seconds late (2026-10-10: 1.8 Mbps -> RTT up to 9.5 s).
This node watches that queue directly and backs the video off.

  ping <viewer> 5x/s over Tailscale (same path as the video)
    -> queue delay = smoothed RTT - baseline RTT (min over 120 s)
    -> delay-based AIMD on one scale factor
    -> /webrtc/bitrate_scale (std_msgs/Float32)
  every webrtc_node: NVENC bitrate = its bitrate_bps * scale (live)

One scale for all cameras because they share one uplink, so the
front/side split stays the same as the link grows and shrinks.
Increases only happen while video is actually being sent (uplink tx
near the target): with no viewer the link is idle, RTT looks perfect,
and ramping to max would just overload the link when a viewer joins.

When even the floor scale clogs the link (2026-10-12: ~300 kbps
cellular; NVENC overshoots small CBR targets, so 0.25x of 1.2 Mbps
was really ~480 kbps), the side cameras drop to a still frame every
few seconds (/webrtc/sides_enabled = false) and the front gets the
whole link. They come back once the front alone runs well above what
three cameras would need at the floor.
"""

import collections
import re
import subprocess
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

RTT_RE = re.compile(r'icmp_seq=(\d+).*time=([\d.]+) ms')


def uplink_interface():
    """Interface carrying the default route (the cellular router)."""
    try:
        out = subprocess.run(['ip', 'route', 'get', '8.8.8.8'],
                             capture_output=True, text=True, timeout=2).stdout
        return out.split(' dev ')[1].split()[0]
    except Exception:
        return None


class RateController(Node):
    """Delay-based AIMD on a shared video bitrate scale."""

    def __init__(self):
        super().__init__('webrtc_rate_controller')

        # Host the video goes to (sim chair Tailscale IP).
        self.declare_parameter('target', '100.91.25.107')
        self.declare_parameter('ping_interval_s', 0.2)
        self.declare_parameter('min_scale', 0.25)
        self.declare_parameter('max_scale', 1.5)
        self.declare_parameter('initial_scale', 0.6)
        # Queue delay (ms above baseline RTT) thresholds.
        self.declare_parameter('high_delay_ms', 200.0)
        self.declare_parameter('low_delay_ms', 80.0)
        self.declare_parameter('panic_delay_ms', 800.0)
        self.declare_parameter('decrease_factor', 0.7)
        self.declare_parameter('increase_per_s', 0.02)
        # Total video bitrate at scale 1.0 (sum of the encoders'
        # bitrate_bps), used to tell "sending video" from "idle".
        self.declare_parameter('base_total_bps', 1200000)
        # Front camera bitrate_bps: the load left when the sides pause.
        self.declare_parameter('front_bps', 500000)
        # Pause the sides after this long at min_scale with the link
        # still clogged; resume after this long calm with headroom.
        self.declare_parameter('side_pause_after_s', 3.0)
        self.declare_parameter('side_resume_hold_s', 15.0)

        p = lambda n: self.get_parameter(n).value  # noqa: E731
        self.target = p('target')
        self.interval = float(p('ping_interval_s'))
        self.min_scale = float(p('min_scale'))
        self.max_scale = float(p('max_scale'))
        self.scale = float(p('initial_scale'))
        self.high = float(p('high_delay_ms'))
        self.low = float(p('low_delay_ms'))
        self.panic = float(p('panic_delay_ms'))
        self.dec = float(p('decrease_factor'))
        self.inc = float(p('increase_per_s'))
        self.base_total = float(p('base_total_bps'))
        self.front = float(p('front_bps'))
        self.pause_after = float(p('side_pause_after_s'))
        self.resume_hold = float(p('side_resume_hold_s'))

        self._lock = threading.Lock()
        self._rtts = collections.deque()    # (t, rtt_ms), last 120 s
        self._replies = collections.deque()  # t of replies, last 2 s
        self._last_reply = time.monotonic()
        self._last_decrease = 0.0
        self._calm_since = None
        self._iface = uplink_interface()
        self._tx = None                     # (t, tx_bytes)
        self._tx_bps = 0.0
        self.sides = True
        self._floor_bad_since = None

        self._pub = self.create_publisher(Float32, '/webrtc/bitrate_scale', 1)
        self._sides_pub = self.create_publisher(Bool, '/webrtc/sides_enabled', 1)
        threading.Thread(target=self._ping_loop, daemon=True).start()
        self.create_timer(0.5, self._step)
        self.create_timer(5.0, self._log)
        self._last_state = ''

        self.get_logger().info(
            f'Rate controller: ping {self.target} every {self.interval:g} s, '
            f'uplink {self._iface}, scale {self.min_scale:g}..'
            f'{self.max_scale:g} (start {self.scale:g})')

    # ----- measurement ----------------------------------------------------

    def _ping_loop(self):
        cmd = ['ping', '-n', '-i', str(self.interval), '-W', '2', self.target]
        while rclpy.ok():
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                    stderr=subprocess.DEVNULL, text=True)
            for line in proc.stdout:
                m = RTT_RE.search(line)
                if not m:
                    continue
                now = time.monotonic()
                with self._lock:
                    self._rtts.append((now, float(m.group(2))))
                    self._replies.append(now)
                    self._last_reply = now
            proc.wait()
            time.sleep(1.0)   # ping exited (e.g. no route yet); restart

    def _uplink_bps(self, now):
        if not self._iface:
            return None
        try:
            with open(f'/sys/class/net/{self._iface}/statistics/tx_bytes') as f:
                tx = int(f.read())
        except OSError:
            return None
        if self._tx is not None and now > self._tx[0]:
            bps = (tx - self._tx[1]) * 8 / (now - self._tx[0])
            self._tx_bps = 0.5 * self._tx_bps + 0.5 * bps
        self._tx = (now, tx)
        return self._tx_bps

    # ----- control --------------------------------------------------------

    def _step(self):
        now = time.monotonic()
        with self._lock:
            while self._rtts and now - self._rtts[0][0] > 120.0:
                self._rtts.popleft()
            while self._replies and now - self._replies[0] > 2.0:
                self._replies.popleft()
            rtts = list(self._rtts)
            replies = len(self._replies)
            silent = now - self._last_reply
        tx_bps = self._uplink_bps(now)
        up = f', uplink {tx_bps / 1000:.0f} kbps' if tx_bps is not None else ''

        expected = 2.0 / self.interval
        if silent > 2.0:
            self._last_state = f'no ping reply for {silent:.0f} s' + up
            self._set(self.min_scale, now, decrease=True)
            self._maybe_pause_sides(now, True)
        elif not rtts:
            self._last_state = 'waiting for first ping'
        else:
            base = min(r for _, r in rtts)
            recent = sorted(r for t, r in rtts if now - t <= 1.0) or [rtts[-1][1]]
            delay = recent[len(recent) // 2] - base     # median, last 1 s
            # Ignore the first 2 s (window not full yet).
            lossy = (now - rtts[0][0] >= 2.0
                     and replies < 0.6 * expected)
            self._last_state = (
                f'rtt base {base:.0f} ms, queue {delay:.0f} ms, '
                f'replies {replies}/{expected:.0f}' + up)
            clogged = delay > self.high or lossy
            if delay > self.panic:
                self._set(self.min_scale, now, decrease=True)
                self._calm_since = None
            elif clogged:
                # One cut per second: the queue needs time to drain
                # before the RTT reflects the new rate.
                if now - self._last_decrease >= 1.0:
                    self._set(self.scale * self.dec, now, decrease=True)
                self._calm_since = None
            elif delay < self.low:
                if self._calm_since is None:
                    self._calm_since = now
                target = self.scale * (self.base_total if self.sides else self.front)
                sending = tx_bps is None or tx_bps >= 0.5 * target
                if (now - self._calm_since >= 2.0 and sending
                        and now - self._last_decrease >= 5.0):
                    self._set(self.scale * (1 + self.inc * 0.5), now)
                self._maybe_resume_sides(now)
            else:
                self._calm_since = None   # in the band: hold
            self._maybe_pause_sides(now, clogged or delay > self.panic)

        sides = Bool()
        sides.data = self.sides
        self._sides_pub.publish(sides)

        msg = Float32()
        msg.data = float(self.scale)
        self._pub.publish(msg)

    def _maybe_pause_sides(self, now, clogged):
        at_floor = self.scale <= self.min_scale * 1.001
        if not (self.sides and at_floor and clogged):
            self._floor_bad_since = None
            return
        if self._floor_bad_since is None:
            self._floor_bad_since = now
        elif now - self._floor_bad_since >= self.pause_after:
            self.sides = False
            self._floor_bad_since = None
            self.get_logger().warn(
                f'side cameras PAUSED: still clogged at the floor '
                f'({self._last_state})')

    def _maybe_resume_sides(self, now):
        # Resume only if the front alone carries comfortably more than
        # all three cameras need at the floor; keep the total load the
        # same across the switch by scaling down for the added sides.
        front_load = self.scale * self.front
        need = 1.3 * self.min_scale * self.base_total
        if (self.sides or front_load < need
                or now - self._calm_since < self.resume_hold
                or now - self._last_decrease < self.resume_hold):
            return
        self.sides = True
        self.scale = max(self.min_scale, front_load / self.base_total)
        self._last_decrease = now   # hold before probing up again
        self.get_logger().warn(
            f'side cameras RESUMED at scale {self.scale:.2f} '
            f'({self._last_state})')

    def _set(self, scale, now, decrease=False):
        scale = min(self.max_scale, max(self.min_scale, scale))
        if decrease and scale < self.scale:
            self._last_decrease = now
            self.get_logger().warn(
                f'scale {self.scale:.2f} -> {scale:.2f} ({self._last_state})')
        self.scale = scale

    def _log(self):
        load = self.scale * (self.base_total if self.sides else self.front)
        self.get_logger().info(
            f'scale {self.scale:.2f} (~{load / 1000:.0f} kbps, '
            f'sides {"on" if self.sides else "PAUSED"}); {self._last_state}')


def main(args=None):
    rclpy.init(args=args)
    node = RateController()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
