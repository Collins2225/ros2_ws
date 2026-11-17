#!/usr/bin/env python3
"""plot_house.py <scenario.json> <outdir> <algo>...
Reads  <outdir>/raw/<algo>.csv  (t,x,y,yaw from /odom)  and draws how each robot moved through the house.
Writes  house_<algo>.png  house_all.png  house_overlay.png  house_diagrams.pdf  into <outdir>."""
import csv, json, math, os, sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.patches import Polygon, Circle
from matplotlib.backends.backend_pdf import PdfPages

NAMES = {'vfh': 'VFH', 'vfh_plus': 'VFH+', 'nearness_diagram': 'Nearness Diagram'}
COLORS = {'vfh': '#c8344c', 'vfh_plus': '#3550d6', 'nearness_diagram': '#d99100'}
ROBOT_L, ROBOT_W = 0.21, 0.164
scen_file, outdir, algos = sys.argv[1], sys.argv[2], sys.argv[3:]
S = json.load(open(scen_file))
goal, start = S['goal'], S['start']

def box_poly(o):
    c, s = math.cos(o['yaw']), math.sin(o['yaw'])
    pts = [(-o['sx'] / 2, -o['sy'] / 2), (o['sx'] / 2, -o['sy'] / 2), (o['sx'] / 2, o['sy'] / 2), (-o['sx'] / 2, o['sy'] / 2)]
    return [(o['x'] + px * c - py * s, o['y'] + px * s + py * c) for px, py in pts]

def dist_to(o, x, y):
    if o['shape'] == 'circle':
        return max(math.hypot(x - o['x'], y - o['y']) - o['r'], 0.0)
    c, s = math.cos(-o['yaw']), math.sin(-o['yaw'])
    dx, dy = x - o['x'], y - o['y']
    px, py = dx * c - dy * s, dx * s + dy * c
    return math.hypot(max(abs(px) - o['sx'] / 2, 0), max(abs(py) - o['sy'] / 2, 0))

def load(a):
    f = os.path.join(outdir, 'raw', a + '.csv')
    if not os.path.exists(f):
        return None
    rows = [r for r in csv.DictReader(open(f))]
    if len(rows) < 5:
        return None
    t = [float(r['t']) for r in rows]; x = [float(r['x']) for r in rows]; y = [float(r['y']) for r in rows]; yaw = [float(r['yaw']) for r in rows]
    t0 = t[0]
    return [v - t0 for v in t], x, y, yaw

def stats(d):
    t, x, y, yaw = d
    length = sum(math.hypot(x[i] - x[i - 1], y[i] - y[i - 1]) for i in range(1, len(x)))
    best, bi = 9.0, 0
    for i in range(len(x)):
        m = min(dist_to(o, x[i], y[i]) for o in S['obstacles'])
        if m < best: best, bi = m, i
    reached = math.hypot(x[-1] - goal[0], y[-1] - goal[1]) < 0.35
    return dict(time=t[-1], length=length, closest=best, ci=bi, reached=reached)

def draw_house(ax):
    for o in S['obstacles']:
        wall = o['name'].startswith('wall')
        fc = '#59636e' if wall else '#d5dbe1'
        if o['shape'] == 'circle':
            ax.add_patch(Circle((o['x'], o['y']), o['r'], fc='#b8d8c0', ec='#4d6b57', lw=0.8, zorder=2))
        else:
            ax.add_patch(Polygon(box_poly(o), closed=True, fc=fc, ec='#3a4753', lw=0.8, zorder=2))
    labels = {'sofa_seat': 'sofa', 'coffee_table': 'coffee\ntable', 'armchair_seat': 'armchair', 'sideboard': 'sideboard', 'bookshelf': 'bookshelf',
              'kitchen_island': 'kitchen island', 'refrigerator': 'fridge', 'bed_seat': 'bed', 'wardrobe': 'wardrobe', 'divider': 'divider',
              'ottoman': 'ottoman', 'cardboard_1': 'box', 'cardboard_2': 'box'}
    for o in S['obstacles']:
        if o['name'] in labels:
            ax.text(o['x'], o['y'], labels[o['name']], ha='center', va='center', fontsize=6.5, color='#2b3640', zorder=3)
    xs = [o['x'] for o in S['obstacles'] if o['name'].startswith('wall')]
    ax.set_xlim(min(xs) - 0.25, max(xs) + 0.25)
    ys = [o['y'] for o in S['obstacles'] if o['name'].startswith('wall')]
    ax.set_ylim(min(ys) - 0.25, max(ys) + 0.25)
    ax.set_aspect('equal'); ax.set_xlabel('metres'); ax.set_ylabel('metres')
    ax.grid(True, color='#e4e8ec', lw=0.5, zorder=0)
    ax.plot(start[0], start[1], 'o', color='#14806f', ms=9, zorder=6)
    ax.annotate('start', (start[0], start[1]), (start[0] + 0.1, start[1] + 0.38), fontsize=8, color='#14806f', zorder=7)
    ax.plot(goal[0], goal[1], '*', color='#3550d6', ms=16, zorder=6, mec='white')
    ax.annotate('goal', (goal[0], goal[1]), (goal[0] - 0.55, goal[1] + 0.3), fontsize=8, color='#3550d6', zorder=7)

def draw_path(ax, a, d, st, colored=True):
    t, x, y, yaw = d
    if colored:
        pts = list(zip(x, y)); segs = [[pts[i], pts[i + 1]] for i in range(len(pts) - 1)]
        lc = LineCollection(segs, cmap='viridis', zorder=4, linewidths=2.6); lc.set_array([(t[i] + t[i + 1]) / 2 for i in range(len(segs))])
        ax.add_collection(lc)
    else:
        ax.plot(x, y, color=COLORS[a], lw=2.4, zorder=4, label=NAMES[a])
    # robot outline and time label every 10 s
    nxt = 10.0
    for i in range(len(t)):
        if t[i] >= nxt:
            c, s = math.cos(yaw[i]), math.sin(yaw[i])
            body = [(-ROBOT_L / 2, -ROBOT_W / 2), (ROBOT_L / 2, -ROBOT_W / 2), (ROBOT_L / 2, ROBOT_W / 2), (-ROBOT_L / 2, ROBOT_W / 2)]
            ax.add_patch(Polygon([(x[i] + px * c - py * s, y[i] + px * s + py * c) for px, py in body], closed=True, fc='white', ec='#15202b', lw=0.8, zorder=5))
            ax.plot([x[i], x[i] + 0.2 * c], [y[i], y[i] + 0.2 * s], color='#15202b', lw=1, zorder=5)
            if colored:
                ax.annotate(f'{int(nxt)} s', (x[i], y[i]), (x[i] + 0.06, y[i] - 0.2), fontsize=6.5, color='#15202b', zorder=7)
            nxt += 10.0
    ci = st['ci']
    ax.add_patch(Circle((x[ci], y[ci]), 0.12, fc='none', ec='#c8344c', lw=1.4, ls='--', zorder=6))

def caption(a, st):
    res = 'reached the goal' if st['reached'] else 'did NOT reach the goal'
    return f"{NAMES[a]}: {res} in {st['time']:.0f} s\npath {st['length']:.1f} m, closest approach to an obstacle {st['closest'] * 100:.0f} cm"

data = {a: load(a) for a in algos}
data = {a: d for a, d in data.items() if d}
if not data:
    sys.exit('no recorded runs found in ' + os.path.join(outdir, 'raw'))
st = {a: stats(d) for a, d in data.items()}

# one picture per algorithm
for a, d in data.items():
    fig, ax = plt.subplots(figsize=(8, 8.4))
    draw_house(ax); draw_path(ax, a, d, st[a])
    ax.set_title(caption(a, st[a]), fontsize=11, loc='left')
    sm = plt.cm.ScalarMappable(cmap='viridis', norm=plt.Normalize(0, st[a]['time'])); cb = fig.colorbar(sm, ax=ax, fraction=0.04, pad=0.02); cb.set_label('time (s)')
    fig.text(0.01, 0.01, 'red dashed circle = moment the robot was closest to something. White boxes = robot every 10 s.', fontsize=7.5, color='#566573')
    fig.tight_layout(); fig.savefig(os.path.join(outdir, f'house_{a}.png'), dpi=170); plt.close(fig)

# all side by side
n = len(data)
fig, axs = plt.subplots(1, n, figsize=(6.4 * n, 6.9), squeeze=False)
for ax, (a, d) in zip(axs[0], data.items()):
    draw_house(ax); draw_path(ax, a, d, st[a]); ax.set_title(caption(a, st[a]), fontsize=10, loc='left')
fig.suptitle('How each robot moved through the house (colour = time, white box = robot every 10 s)', fontsize=12)
fig.tight_layout(); fig.savefig(os.path.join(outdir, 'house_all.png'), dpi=150)
pdf = PdfPages(os.path.join(outdir, 'house_diagrams.pdf')); pdf.savefig(fig); plt.close(fig)

# overlay
fig, ax = plt.subplots(figsize=(9, 9.2))
draw_house(ax)
for a, d in data.items():
    draw_path(ax, a, d, st[a], colored=False)
ax.legend(loc='upper right', fontsize=9, framealpha=.95)
ax.set_title('All paths on top of each other', fontsize=12, loc='left')
fig.tight_layout(); fig.savefig(os.path.join(outdir, 'house_overlay.png'), dpi=160); pdf.savefig(fig); plt.close(fig)
pdf.close()
print('saved:')
for a in data: print('  ' + caption(a, st[a]).replace('\n', ' | '))
