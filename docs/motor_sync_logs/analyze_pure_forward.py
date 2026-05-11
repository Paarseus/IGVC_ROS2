"""Filter wheel_debug to ticks where commanded steering is ~0 (pure forward intent)."""
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

CSV = Path(sys.argv[1] if len(sys.argv) > 1 else 'csv/avros_wheel_debug.csv')
OUT = Path(sys.argv[2] if len(sys.argv) > 2 else '.')
df = pd.read_csv(CSV)
t = df['t_rel'].values

# Pure forward: |w_target| ~ 0 AND |v_target| > 0.1 m/s
pure = df[(df['w_target'].abs() < 0.05) & (df['v_target'].abs() > 0.1)].copy()
print(f'Pure-forward/reverse ticks (no steer): {len(pure)} / {len(df)}')
if len(pure) < 30:
    print('Not enough pure-forward ticks to analyze. Re-record with longer straight segments.')
    sys.exit(0)

# At pure-forward intent the diff-drive math demands L_cmd == R_cmd exactly.
# So any L_meas != R_meas is purely the per-wheel tracking discrepancy.
pure['cmd_match'] = pure['L_cmd_rpm'] - pure['R_cmd_rpm']
pure['meas_diff'] = pure['L_meas_rpm'] - pure['R_meas_rpm']
print()
print('At pure forward (steer=0): commanded L should equal commanded R')
print(f'  L_cmd - R_cmd:  mean={pure["cmd_match"].mean():.2f}  max|.|={pure["cmd_match"].abs().max():.2f}')
print()
print('At pure forward: measured L vs R asymmetry')
print(f'  L_meas - R_meas:  mean={pure["meas_diff"].mean():+.1f}  median={pure["meas_diff"].median():+.1f}  std={pure["meas_diff"].std():.1f}')
print(f'  max L > R by:  {pure["meas_diff"].max():+.1f}')
print(f'  max R > L by:  {pure["meas_diff"].min():+.1f}')

# Per-wheel ratio at pure-forward
pure['L_ratio'] = pure['L_meas_rpm'] / pure['L_cmd_rpm'].replace(0, np.nan)
pure['R_ratio'] = pure['R_meas_rpm'] / pure['R_cmd_rpm'].replace(0, np.nan)
print()
print('Tracking ratio (meas/cmd) under pure-forward intent:')
print(f'  L: mean={pure["L_ratio"].mean():+.3f}  median={pure["L_ratio"].median():+.3f}')
print(f'  R: mean={pure["R_ratio"].mean():+.3f}  median={pure["R_ratio"].median():+.3f}')

# Plot: L_meas vs R_meas under pure forward intent
fig, axes = plt.subplots(2, 1, figsize=(14, 7), sharex=True)
ax = axes[0]
ax.plot(pure['t_rel'], pure['L_cmd_rpm'], label='L_cmd', color='tab:blue', alpha=0.5)
ax.plot(pure['t_rel'], pure['L_meas_rpm'], label='L_meas', color='tab:blue', linestyle='--')
ax.plot(pure['t_rel'], pure['R_cmd_rpm'], label='R_cmd', color='tab:red', alpha=0.5)
ax.plot(pure['t_rel'], pure['R_meas_rpm'], label='R_meas', color='tab:red', linestyle='--')
ax.set_ylabel('motor RPM')
ax.set_title(f'Pure-forward window only (steer=0, |v|>0.1 m/s) — {len(pure)} ticks')
ax.legend(loc='upper right', ncol=4, fontsize=9)
ax.grid(alpha=0.3)
ax.axhline(0, color='k', lw=0.5)

ax = axes[1]
ax.plot(pure['t_rel'], pure['meas_diff'], color='tab:purple', label='L_meas - R_meas')
ax.set_ylabel('L − R measured RPM')
ax.set_xlabel('time (s)')
ax.set_title('Per-wheel asymmetry under pure-forward command')
ax.legend(loc='upper right')
ax.grid(alpha=0.3)
ax.axhline(0, color='k', lw=0.5)

fig.tight_layout()
out = OUT / 'wheel_sync_pure_forward.png'
fig.savefig(out, dpi=110)
print(f'\nSaved: {out}')
