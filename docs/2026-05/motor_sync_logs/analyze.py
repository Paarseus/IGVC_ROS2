"""Compare L vs R commanded vs measured RPMs and positions from wheel_debug CSV."""
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

CSV = Path(sys.argv[1] if len(sys.argv) > 1 else 'csv/avros_wheel_debug.csv')
OUT = Path(sys.argv[2] if len(sys.argv) > 2 else '.')

df = pd.read_csv(CSV)
t = df['t_rel'].values

# Normalize positions to start at 0 for the bag window so deltas are visible
df['L_pos_rel'] = df['L_pos_rev'] - df['L_pos_rev'].iloc[0]
df['R_pos_rel'] = df['R_pos_rev'] - df['R_pos_rev'].iloc[0]

# Per-tick command/meas error per wheel
df['L_err'] = df['L_meas_rpm'] - df['L_cmd_rpm']
df['R_err'] = df['R_meas_rpm'] - df['R_cmd_rpm']
# Cross-wheel asymmetry (does R follow L when both should be equal?)
df['cmd_diff'] = df['R_cmd_rpm'] - df['L_cmd_rpm']
df['meas_diff'] = df['R_meas_rpm'] - df['L_meas_rpm']

# Print summary stats
print('=' * 70)
print(f'Window: {t[0]:.2f}s  to  {t[-1]:.2f}s  ({len(df)} samples @ ~50Hz)')
print('=' * 70)
print()
print('Cumulative position over the bag window (motor revs):')
print(f'  L delta = {df["L_pos_rel"].iloc[-1]:+.2f} rev')
print(f'  R delta = {df["R_pos_rel"].iloc[-1]:+.2f} rev')
print(f'  ratio    R/L  = {df["R_pos_rel"].iloc[-1] / df["L_pos_rel"].iloc[-1]:+.3f}')
print()

# Active driving subset (avoid divide-by-zero on idle ticks)
active = df[df['L_cmd_rpm'].abs() > 100].copy()
print(f'Active ticks (|L_cmd| > 100 RPM): {len(active)} / {len(df)}')
if len(active) > 50:
    active['ratio_meas_L'] = active['L_meas_rpm'] / active['L_cmd_rpm']
    active['ratio_meas_R'] = active['R_meas_rpm'] / active['R_cmd_rpm']
    print(f'  L follow-ratio (meas/cmd):  mean={active["ratio_meas_L"].mean():+.3f}  median={active["ratio_meas_L"].median():+.3f}  std={active["ratio_meas_L"].std():.3f}')
    print(f'  R follow-ratio (meas/cmd):  mean={active["ratio_meas_R"].mean():+.3f}  median={active["ratio_meas_R"].median():+.3f}  std={active["ratio_meas_R"].std():.3f}')
    print()
    print('Per-wheel sign correlation when both commanded same sign:')
    same_sign = active[np.sign(active['L_cmd_rpm']) == np.sign(active['R_cmd_rpm'])]
    print(f'  ticks with both cmds same sign: {len(same_sign)}')
    if len(same_sign):
        l_sign_match = (np.sign(same_sign['L_meas_rpm']) == np.sign(same_sign['L_cmd_rpm'])).mean()
        r_sign_match = (np.sign(same_sign['R_meas_rpm']) == np.sign(same_sign['R_cmd_rpm'])).mean()
        print(f'  fraction L_meas sign matches L_cmd sign: {l_sign_match:.3f}')
        print(f'  fraction R_meas sign matches R_cmd sign: {r_sign_match:.3f}')
        print(f'  fraction R_meas sign matches L_cmd sign: '
              f'{(np.sign(same_sign["R_meas_rpm"]) == np.sign(same_sign["L_cmd_rpm"])).mean():.3f}')

# === Plots ===
fig, axes = plt.subplots(4, 1, figsize=(14, 12), sharex=True)

ax = axes[0]
ax.plot(t, df['L_cmd_rpm'], label='L cmd', color='tab:blue', alpha=0.6)
ax.plot(t, df['L_meas_rpm'], label='L meas', color='tab:blue', linestyle='--')
ax.plot(t, df['R_cmd_rpm'], label='R cmd', color='tab:red', alpha=0.6)
ax.plot(t, df['R_meas_rpm'], label='R meas', color='tab:red', linestyle='--')
ax.set_ylabel('motor RPM')
ax.set_title('Commanded vs Measured per wheel')
ax.legend(loc='upper right', ncol=4, fontsize=9)
ax.grid(alpha=0.3)
ax.axhline(0, color='k', lw=0.5)

ax = axes[1]
ax.plot(t, df['cmd_diff'], label='R_cmd − L_cmd', color='tab:purple', alpha=0.7)
ax.plot(t, df['meas_diff'], label='R_meas − L_meas', color='tab:orange')
ax.set_ylabel('Δ RPM (R−L)')
ax.set_title('Asymmetry: difference between wheels (cmd vs meas)')
ax.legend(loc='upper right', fontsize=9)
ax.grid(alpha=0.3)
ax.axhline(0, color='k', lw=0.5)

ax = axes[2]
ax.plot(t, df['L_err'], label='L err (meas−cmd)', color='tab:blue', alpha=0.7)
ax.plot(t, df['R_err'], label='R err (meas−cmd)', color='tab:red', alpha=0.7)
ax.set_ylabel('tracking error (RPM)')
ax.set_title('Per-wheel SparkMAX PID tracking error')
ax.legend(loc='upper right', fontsize=9)
ax.grid(alpha=0.3)
ax.axhline(0, color='k', lw=0.5)

ax = axes[3]
ax.plot(t, df['L_pos_rel'], label='L pos (relative, motor revs)', color='tab:blue')
ax.plot(t, df['R_pos_rel'], label='R pos (relative, motor revs)', color='tab:red')
ax.set_ylabel('cumulative motor revs')
ax.set_xlabel('time (s)')
ax.set_title('Integrated wheel position over bag window')
ax.legend(loc='upper right', fontsize=9)
ax.grid(alpha=0.3)
ax.axhline(0, color='k', lw=0.5)

fig.tight_layout()
out_path = OUT / 'wheel_sync.png'
fig.savefig(out_path, dpi=110)
print(f'\nSaved plot: {out_path}')

# Side-by-side scatter: L_cmd vs L_meas, R_cmd vs R_meas
fig2, (a1, a2) = plt.subplots(1, 2, figsize=(12, 5))
a1.scatter(df['L_cmd_rpm'], df['L_meas_rpm'], s=4, alpha=0.4, color='tab:blue')
lim = max(df['L_cmd_rpm'].abs().max(), df['L_meas_rpm'].abs().max())
a1.plot([-lim, lim], [-lim, lim], 'k--', alpha=0.4, label='ideal y=x')
a1.set_xlabel('L_cmd_rpm')
a1.set_ylabel('L_meas_rpm')
a1.set_title('L wheel: cmd → meas')
a1.legend()
a1.grid(alpha=0.3)
a1.axhline(0, color='k', lw=0.3)
a1.axvline(0, color='k', lw=0.3)

a2.scatter(df['R_cmd_rpm'], df['R_meas_rpm'], s=4, alpha=0.4, color='tab:red')
lim = max(df['R_cmd_rpm'].abs().max(), df['R_meas_rpm'].abs().max())
a2.plot([-lim, lim], [-lim, lim], 'k--', alpha=0.4, label='ideal y=x')
a2.set_xlabel('R_cmd_rpm')
a2.set_ylabel('R_meas_rpm')
a2.set_title('R wheel: cmd → meas')
a2.legend()
a2.grid(alpha=0.3)
a2.axhline(0, color='k', lw=0.3)
a2.axvline(0, color='k', lw=0.3)

fig2.tight_layout()
out2 = OUT / 'wheel_cmd_vs_meas_scatter.png'
fig2.savefig(out2, dpi=110)
print(f'Saved plot: {out2}')
