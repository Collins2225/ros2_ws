"""Teaches the referee and the launch file about extra rooms. Safe to run twice: it skips what is done.
  1. judge.py       : new parameter scenario_file (a .json with the obstacles); empty = the old built-in arena
  2. sim.launch.py  : the environment variable OBS_AVOID_WORLD may name another world file
"""
import os
import shutil
import sys
import time

src = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else '~/ros2_ws/src')
backup_dir = os.path.expanduser(sys.argv[2] if len(sys.argv) > 2 else '~/ros2_ws/project/backup_before_rooms')
stamp = time.strftime('%Y%m%d_%H%M%S')
os.makedirs(backup_dir, exist_ok=True)
failed = False


def patch(path, label, already, replacements):
    """replacements: list of (old, new). Every 'old' must be found exactly once, or nothing is changed."""
    global failed
    if not os.path.isfile(path):
        print(f'{label}: FILE NOT FOUND: {path}')
        failed = True
        return
    text = open(path).read()
    if already in text:
        print(f'{label}: already done, skipped')
        return
    for old, _ in replacements:
        if text.count(old) != 1:
            print(f'{label}: could not find the place to edit (looked for: {old.strip()[:60]!r}). NOTHING CHANGED.')
            failed = True
            return
    for old, new in replacements:
        text = text.replace(old, new)
    shutil.copy2(path, os.path.join(backup_dir, os.path.basename(path) + '.' + stamp))
    open(path, 'w').write(text)
    try:
        compile(text, path, 'exec')
    except SyntaxError as err:
        shutil.copy2(os.path.join(backup_dir, os.path.basename(path) + '.' + stamp), path)
        print(f'{label}: the result had a syntax error ({err}); the original was restored')
        failed = True
        return
    print(f'{label}: done')


patch(os.path.join(src, 'obs_avoid_control', 'obs_avoid_control', 'metrics', 'judge.py'), 'referee (judge.py)',
      'scenario_file',
      [("        self.declare_parameter('csv_path', '~/ros2_ws/project/results/raw/runs.csv')\n",
        "        self.declare_parameter('csv_path', '~/ros2_ws/project/results/raw/runs.csv')\n"
        "        self.declare_parameter('scenario_file', '')    # a .json with the obstacles; empty = the built-in arena\n"),
       ("        self.csv_path = os.path.expanduser(g('csv_path').value)\n",
        "        self.csv_path = os.path.expanduser(g('csv_path').value)\n"
        "        self.obstacles = OBSTACLES\n"
        "        scenario_file = os.path.expanduser(g('scenario_file').value)\n"
        "        if scenario_file:\n"
        "            import json\n"
        "            with open(scenario_file) as f:\n"
        "                self.obstacles = json.load(f)['obstacles']\n"
        "            self.get_logger().info(f'obstacles from {scenario_file}: {len(self.obstacles)} objects')\n"),
       ("        for o in OBSTACLES:\n", "        for o in self.obstacles:\n")])

patch(os.path.join(src, 'obs_avoid_description', 'launch', 'sim.launch.py'), 'launch file (sim.launch.py)',
      'OBS_AVOID_WORLD',
      [("    world = os.path.join(pkg, 'worlds', 'obstacle_course.sdf')\n",
        "    world = os.environ.get('OBS_AVOID_WORLD') or os.path.join(pkg, 'worlds', 'obstacle_course.sdf')\n")])

print('ALL GOOD' if not failed else 'SOMETHING FAILED, read the lines above')
sys.exit(1 if failed else 0)
