import csv
import glob
import os
import sys

import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Polygon

# We borrow the referee's own map and robot outline, so the picture can never disagree with the scoring.
from obs_avoid_control.metrics.judge import (OBSTACLES, ROBOT_X0, ROBOT_X1, ROBOT_Y0, ROBOT_Y1,
                                             box_corners)

GOAL, TOLERANCE = (3.0, 0.0), 0.15
folder = os.path.expanduser('~/ros2_ws/project/results/raw/trajectories')

if len(sys.argv) > 1:
    path = sys.argv[1]
else:
    files = sorted(glob.glob(os.path.join(folder, '*.csv')))
    if not files:
        sys.exit('No path files yet. Run a round first.')
    path = files[-1]                       # the newest one

rows = list(csv.DictReader(open(path)))
t = [float(r['t_s']) for r in rows]
x = [float(r['x_m']) for r in rows]
y = [float(r['y_m']) for r in rows]
yaw = [float(r['yaw_rad']) for r in rows]
gap = [float(r['gap_m']) for r in rows]
closest = min(range(len(gap)), key=lambda i: gap[i])

fig, ax = plt.subplots(figsize=(9, 5.5))

for o in OBSTACLES:                         # the obstacles, in grey
    if o['shape'] == 'box':
        pts = box_corners(o['x'], o['y'], -o['sx'] / 2, o['sx'] / 2, -o['sy'] / 2, o['sy'] / 2, o['yaw'])
        ax.add_patch(Polygon(pts, closed=True, color='0.6'))
    else:
        ax.add_patch(Circle((o['x'], o['y']), o['r'], color='0.6'))
    ax.annotate(o['name'], (o['x'], o['y']), ha='center', va='center', fontsize=8)

ax.add_patch(Circle(GOAL, TOLERANCE, fill=False, edgecolor='green', linestyle='--'))   # the goal zone
ax.plot(*GOAL, marker='*', color='green', markersize=12, label='goal')
ax.plot(x[0], y[0], marker='o', color='black', label='start')

dots = ax.scatter(x, y, c=gap, cmap='viridis_r', s=14, vmin=0, vmax=0.6)               # the path
fig.colorbar(dots, ax=ax, label='gap to the nearest obstacle (m): purple = close')

robot = box_corners(x[closest], y[closest], ROBOT_X0, ROBOT_X1, ROBOT_Y0, ROBOT_Y1, yaw[closest])
ax.add_patch(Polygon(robot, closed=True, fill=False, edgecolor='red', linewidth=1.5))  # the robot at its closest moment
ax.annotate(f'closest: {gap[closest] * 100:.1f} cm\nat {t[closest]:.1f} s', (x[closest], y[closest]),
            xytext=(x[closest] - 0.3, y[closest] - 0.55), color='red', fontsize=9,
            arrowprops=dict(arrowstyle='->', color='red'))

ax.set_xlim(-0.3, 3.9)
ax.set_ylim(-1.7, 1.7)
ax.set_aspect('equal')
ax.set_xlabel('x (m): forward')
ax.set_ylabel('y (m): left')
ax.set_title(os.path.basename(path))
ax.legend(loc='upper left')

out_dir = os.path.expanduser('~/ros2_ws/project/results/plots')
os.makedirs(out_dir, exist_ok=True)
out = os.path.join(out_dir, os.path.basename(path).replace('.csv', '.png'))
fig.savefig(out, dpi=150, bbox_inches='tight')
print('picture saved to', out)
plt.show()
