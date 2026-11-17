#!/usr/bin/env python3
"""fancy_world.py - the living room with real 3D furniture from Gazebo Fuel. FOR DEMOS ONLY.

    ~/fuel_venv/bin/python fancy_world.py

What it does
  * keeps everything solid exactly where it is (the same invisible collision shapes, the same referee file),
  * swaps the plain boxes you SEE for real 3D models that you downloaded with fuel_get.sh,
  * scales each model to the footprint of the piece it replaces, so nothing sticks into anything.
It writes  living_room_fancy.sdf  and  living_room_fancy.json  next to this file.

Which model goes where is in  fancy_models.json  (made on the first run). To fix a model that faces the wrong
way, change its "turn" (degrees: 0, 90, 180 or 270) and run this file again. No rebuild is needed.
The real furniture has different outlines than my boxes, so the sonar sees something slightly different:
use this room to SHOW the robot, and the plain living_room for the numbers in the thesis.
"""
import json
import math
import os
import re
import sys
import xml.etree.ElementTree as ET

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import make_world as mw                      # noqa: E402  (the plain room: furniture list, decorations, world text)

NAME = 'living_room_fancy'
CONFIG = os.path.join(HERE, 'fancy_models.json')

# Each piece of furniture that can be swapped:  where it stands, the footprint (x, y) and height it must fit,
# the direction it faces (degrees, 0 = +x, 90 = +y, -90 = -y), which plain parts it replaces.
PIECES = {
    'sofa':         dict(x=1.0,  y=1.55,  lx=1.80, ly=0.80, h=0.85, face=-90, fit='stretch',
                         parts=['sofa_seat', 'sofa_back'], decor=['sofa_deco']),
    'armchair':     dict(x=2.6,  y=1.45,  lx=0.80, ly=0.80, h=0.85, face=-90, fit='uniform',
                         parts=['armchair_seat', 'armchair_back'], decor=['armchair_deco']),
    'coffee_table': dict(x=1.0,  y=0.55,  lx=0.90, ly=0.50, h=0.40, face=0,   fit='stretch',
                         parts=['coffee_table'], decor=['coffee_table_deco']),
    'dining_table': dict(x=3.3,  y=-0.90, lx=1.40, ly=0.80, h=0.76, face=0,   fit='stretch',
                         parts=['dining_table_top'] + [f'dining_table_leg{i}' for i in range(1, 5)], decor=[]),
    'chair_1':      dict(x=2.95, y=-0.32, lx=0.46, ly=0.46, h=0.90, face=-90, fit='uniform',
                         parts=['chair_1_seat', 'chair_1_back'] + [f'chair_1_leg{i}' for i in range(1, 5)],
                         decor=['chairs_deco']),
    'chair_2':      dict(x=3.65, y=-0.32, lx=0.46, ly=0.46, h=0.90, face=-90, fit='uniform',
                         parts=['chair_2_seat', 'chair_2_back'] + [f'chair_2_leg{i}' for i in range(1, 5)],
                         decor=['chairs_deco']),
    'chair_3':      dict(x=2.35, y=-0.90, lx=0.46, ly=0.46, h=0.90, face=0,   fit='uniform',
                         parts=['chair_3_seat', 'chair_3_back'] + [f'chair_3_leg{i}' for i in range(1, 5)],
                         decor=['chairs_deco']),
    'bookshelf':    dict(x=5.2,  y=1.0,   lx=0.40, ly=1.20, h=1.30, face=180, fit='stretch',
                         parts=['bookshelf'], decor=['bookshelf_deco']),
    'sideboard':    dict(x=1.2,  y=-1.75, lx=1.40, ly=0.40, h=0.80, face=90,  fit='stretch',
                         parts=['sideboard'], decor=[]),
}

# the positions follow the plain room (so a wider or narrower room needs no change here)
for _k, (_x, _y) in getattr(mw, 'LAYOUT', {}).items():
    if _k in PIECES:
        PIECES[_k]['x'], PIECES[_k]['y'] = _x, _y

DEFAULT_CONFIG = {
    "note": "model = OWNER/NAME of a model you downloaded (empty text = keep the plain shape). turn = extra "
            "rotation in degrees (0, 90, 180, 270). fit = stretch (fill the footprint) or uniform (keep proportions). "
            "grow = extra size factor (1.0 = none). After a change run: ~/fuel_venv/bin/python fancy_world.py",
    "pieces": {
        "sofa":         {"model": "OpenRobotics/Sofa",        "turn": 0, "fit": "stretch", "grow": 1.0},
        "armchair":     {"model": "OpenRobotics/Armchair",    "turn": 0, "fit": "uniform", "grow": 1.0},
        "coffee_table": {"model": "OpenRobotics/CoffeeTable", "turn": 0, "fit": "stretch", "grow": 1.0},
        "dining_table": {"model": "OpenRobotics/Table",       "turn": 0, "fit": "stretch", "grow": 1.0},
        "chair_1":      {"model": "OpenRobotics/Chair",       "turn": 0, "fit": "uniform", "grow": 1.0},
        "chair_2":      {"model": "OpenRobotics/Chair",       "turn": 0, "fit": "uniform", "grow": 1.0},
        "chair_3":      {"model": "OpenRobotics/Chair",       "turn": 0, "fit": "uniform", "grow": 1.0},
        "bookshelf":    {"model": "OpenRobotics/Bookshelf",   "turn": 0, "fit": "stretch", "grow": 1.0},
        "sideboard":    {"model": "OpenRobotics/TVStand",     "turn": 0, "fit": "stretch", "grow": 1.0},
    },
}


# ---------------------------------------------------------------- finding and reading a downloaded model
def cache_roots():
    roots = []
    if os.environ.get('GZ_FUEL_CACHE_PATH'):
        roots.append(os.path.expanduser(os.environ['GZ_FUEL_CACHE_PATH']))
    for base in ('~/.gz/fuel', '~/.ignition/fuel'):
        roots.append(os.path.expanduser(base))
    return roots


def find_model_dir(ref):
    """'OpenRobotics/Sofa' -> the folder with model.sdf of the newest downloaded version, or None."""
    if '/' not in ref:
        return None
    owner, name = ref.split('/', 1)
    for root in cache_roots():
        if not os.path.isdir(root):
            continue
        for server in sorted(os.listdir(root)):
            sdir = os.path.join(root, server)
            if not os.path.isdir(sdir):
                continue
            for o in os.listdir(sdir):
                if o.lower() != owner.lower():
                    continue
                mdir = os.path.join(sdir, o, 'models')
                if not os.path.isdir(mdir):
                    continue
                for m in os.listdir(mdir):
                    if m.lower().replace(' ', '') != name.lower().replace(' ', ''):
                        continue
                    versions = [v for v in os.listdir(os.path.join(mdir, m)) if v.isdigit()]
                    if versions:
                        return os.path.join(mdir, m, max(versions, key=int))
    return None


def parse_pose(text):
    v = [float(t) for t in (text or '0 0 0 0 0 0').split()]
    v += [0.0] * (6 - len(v))
    return v[:6]


def euler_matrix(r, p, y):
    """SDF roll-pitch-yaw -> 3x3 rotation (turn about x, then y, then z, all about the fixed axes)."""
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def rpy_from_matrix(m):
    sp = -m[2, 0]
    p = math.asin(max(-1.0, min(1.0, sp)))
    if abs(sp) < 0.999999:
        return math.atan2(m[2, 1], m[2, 2]), p, math.atan2(m[1, 0], m[0, 0])
    return 0.0, p, math.atan2(-m[0, 1], m[1, 1])


def load_visuals(model_dir):
    """The mesh visuals of a model: file, mesh scale, rotation R, offset t (model frame), material xml."""
    cfg = os.path.join(model_dir, 'model.config')
    sdf_name = 'model.sdf'
    if os.path.isfile(cfg):
        try:
            tag = ET.parse(cfg).getroot().find('sdf')
            if tag is not None and tag.text:
                sdf_name = tag.text.strip()
        except ET.ParseError:
            pass
    root = ET.parse(os.path.join(model_dir, sdf_name)).getroot()
    model = root.find('model') if root.tag == 'sdf' else root
    if model is None:
        raise ValueError('no <model> in ' + sdf_name)

    def pose_of(el):
        p = el.find('pose') if el is not None else None
        return parse_pose(p.text if p is not None else None)

    out, notes = [], []
    mp = pose_of(model)
    for link in model.iter('link'):
        lp = pose_of(link)
        for vis in link.findall('visual'):
            mesh = vis.find('geometry/mesh')
            if mesh is None:
                notes.append('a visual that is not a mesh was skipped')
                continue
            uri = (mesh.findtext('uri') or '').strip()
            sc = [float(t) for t in (mesh.findtext('scale') or '1 1 1').split()]
            vp = pose_of(vis)
            r_m, r_l, r_v = euler_matrix(*mp[3:]), euler_matrix(*lp[3:]), euler_matrix(*vp[3:])
            r = r_m @ r_l @ r_v
            t = r_m @ (r_l @ np.array(vp[:3]) + np.array(lp[:3])) + np.array(mp[:3])
            path = resolve_uri(uri, model_dir)
            if path is None:
                notes.append(f'mesh file not found: {uri}')
                continue
            mat = vis.find('material')
            mat_xml = None
            if mat is not None and (mat.find('pbr') is not None or mat.find('diffuse') is not None
                                    or mat.find('ambient') is not None):
                mat_xml = ET.tostring(mat, encoding='unicode')
            out.append(dict(path=path, scale=np.array(sc), R=r, t=t, material=mat_xml))
    return out, notes


def resolve_uri(uri, model_dir):
    if uri.startswith('file://'):
        p = uri[7:]
        if os.path.isfile(p):
            return p
    elif uri.startswith('model://'):
        parts = uri[8:].split('/', 1)
        if len(parts) == 2 and os.path.isfile(os.path.join(model_dir, parts[1])):
            return os.path.join(model_dir, parts[1])
    elif os.path.isfile(os.path.join(model_dir, uri)):
        return os.path.join(model_dir, uri)
    base = os.path.basename(uri)
    for d, _, files in os.walk(model_dir):
        if base in files:
            return os.path.join(d, base)
    return None


def mesh_vertices(path):
    """All corner points of a mesh file, in metres, z up (collada units and up-axis are applied)."""
    import trimesh
    try:
        obj = trimesh.load(path, skip_materials=True)
    except TypeError:
        obj = trimesh.load(path)
    m = obj.dump(concatenate=True) if isinstance(obj, trimesh.Scene) else obj
    v = np.asarray(m.vertices, dtype=float)
    if path.lower().endswith('.dae'):
        text = open(path, errors='ignore').read(4000)
        unit = re.search(r'<unit[^>]*meter="([^"]+)"', text)
        up = re.search(r'<up_axis>\s*([XYZ])_UP', text)
        if unit:
            v = v * float(unit.group(1))
        if up and up.group(1) == 'Y':
            v = np.column_stack([v[:, 0], -v[:, 2], v[:, 1]])
        elif up and up.group(1) == 'X':
            v = np.column_stack([-v[:, 1], v[:, 0], v[:, 2]])
    return v


# ---------------------------------------------------------------- fitting a model to a piece of furniture
def fit_piece(name, spec, cfg, visuals):
    """Returns (xml of the wrapper model, a one-line report)."""
    verts = []
    for vis in visuals:
        v = mesh_vertices(vis['path']) * vis['scale']
        vis['n'] = len(v)
        verts.append(v @ vis['R'].T + vis['t'])
    allv = np.vstack(verts)
    lo, hi = allv.min(axis=0), allv.max(axis=0)
    bx, by, bz = hi - lo
    centre = np.array([(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, lo[2]])
    if min(bx, by, bz) < 1e-4:
        raise ValueError('the model is flat or empty')

    lx, ly, h = spec['lx'], spec['ly'], spec['h'] * float(cfg.get('grow', 1.0))
    lx, ly = lx * float(cfg.get('grow', 1.0)), ly * float(cfg.get('grow', 1.0))

    def long_axis(a, b):
        return 'x' if a > 1.2 * b else ('y' if b > 1.2 * a else None)

    theta = float(spec['face'])
    swapped = round(theta / 90.0) % 2 != 0
    wx, wy = (by, bx) if swapped else (bx, by)
    ta, ma = long_axis(lx, ly), long_axis(wx, wy)
    if ta and ma and ta != ma:
        theta += 90.0                                          # long side of the model onto the long side of the spot
    theta += float(cfg.get('turn', 0))
    k = theta / 90.0
    axis_aligned = abs(k - round(k)) < 1e-6
    swapped = axis_aligned and round(k) % 2 != 0
    identity = all(np.allclose(v['R'], np.eye(3), atol=1e-6) for v in visuals)

    if swapped:
        sx_, sy_ = ly / bx, lx / by
    else:
        sx_, sy_ = lx / bx, ly / by
    sz_ = h / bz
    mode = cfg.get('fit', spec['fit'])
    note = ''
    if mode == 'stretch' and identity and axis_aligned:
        S = np.array([sx_, sy_, sz_])
    else:
        u = min(sx_, sy_, sz_)
        S = np.array([u, u, u])
        if mode == 'stretch':
            note = ' (model has rotated parts or odd turn, so proportions were kept)'
    warn = ''
    if max(S) > 4 or min(S) < 0.25:
        warn = '  WARNING: very different from the size I expected - check the model'

    vis_xml = ''
    for i, vis in enumerate(visuals):
        if identity and mode == 'stretch' and axis_aligned:
            rot_rpy = (0.0, 0.0, 0.0)
        else:
            rot_rpy = rpy_from_matrix(vis['R'])
        t = S * (vis['t'] - centre)
        sc = S * vis['scale']
        uri = 'file://' + vis['path']
        vis_xml += (f'        <visual name="v{i}"><pose>{t[0]:.5f} {t[1]:.5f} {t[2]:.5f} '
                    f'{rot_rpy[0]:.6f} {rot_rpy[1]:.6f} {rot_rpy[2]:.6f}</pose>'
                    f'<geometry><mesh><uri>{uri}</uri><scale>{sc[0]:.5f} {sc[1]:.5f} {sc[2]:.5f}</scale></mesh></geometry>'
                    f'{vis["material"] or ""}</visual>\n')
    yaw = math.radians(theta)
    xml = (f'    <model name="fancy_{name}"><static>true</static>'
           f'<pose>{spec["x"] + float(cfg.get("dx", 0)):.4f} {spec["y"] + float(cfg.get("dy", 0)):.4f} 0 0 0 {yaw:.6f}</pose>\n'
           f'      <link name="link">\n{vis_xml}      </link></model>\n')
    report = (f'{name:13s} model {bx:.2f} x {by:.2f} x {bz:.2f} m  ->  spot {lx:.2f} x {ly:.2f} x {h:.2f} m, '
              f'turned {theta % 360:.0f} deg, scale ({S[0]:.2f}, {S[1]:.2f}, {S[2]:.2f}){note}{warn}')
    return xml, report


# ---------------------------------------------------------------- main
def main():
    if not os.path.isfile(CONFIG):
        with open(CONFIG, 'w') as f:
            json.dump(DEFAULT_CONFIG, f, indent=1)
        print('made', CONFIG, '(edit it to change models or turn them)')
    with open(CONFIG) as f:
        pieces_cfg = json.load(f).get('pieces', {})

    mw.build_decor()
    drop_decor, hidden, wrappers, reports, replaced = set(), set(), '', [], set()
    for name, spec in PIECES.items():
        cfg = pieces_cfg.get(name, {})
        ref = (cfg.get('model') or '').strip()
        if not ref:
            reports.append(f'{name:13s} kept the plain shape (no model chosen)')
            continue
        mdir = find_model_dir(ref)
        if not mdir:
            reports.append(f'{name:13s} {ref} is NOT downloaded, kept the plain shape')
            continue
        try:
            visuals, notes = load_visuals(mdir)
            if not visuals:
                raise ValueError('no usable mesh in the model ' + ('; '.join(notes)))
            xml, rep = fit_piece(name, spec, cfg, visuals)
        except Exception as err:                                  # one bad model must not stop the others
            reports.append(f'{name:13s} {ref}: could not be used ({err}); kept the plain shape')
            continue
        wrappers += xml
        replaced.add(name)
        hidden.update(spec['parts'])
        drop_decor.update(spec['decor'])
        reports.append(rep + (' | ' + '; '.join(notes) if notes else ''))

    for p in mw.parts:
        if p['name'] in hidden:
            p['visible'] = False
    keep_low = 'sideboard' in replaced
    mw.decor[:] = [d for d in mw.decor
                   if d['group'] not in drop_decor
                   and not (keep_low and d['group'] == 'sideboard_deco' and d['zbase'] < 0.79)]

    sdf = mw.sdf_text().replace(mw.FOOTER, '\n    <!-- real 3D furniture (pictures only; the solid shapes above stay) -->\n'
                                + wrappers + mw.FOOTER)
    with open(os.path.join(HERE, NAME + '.sdf'), 'w') as f:
        f.write(sdf)
    data = json.loads(mw.json_text())
    data['name'] = NAME
    with open(os.path.join(HERE, NAME + '.json'), 'w') as f:
        json.dump(data, f, indent=1)
    print('wrote', os.path.join(HERE, NAME + '.sdf'))
    print('wrote', os.path.join(HERE, NAME + '.json'))
    print()
    for r in reports:
        print(' ', r)
    print()
    print('Start it with:  demo_house_fancy fuzzy')


if __name__ == '__main__':
    main()
