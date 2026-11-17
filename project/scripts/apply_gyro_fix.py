"""Adds a gyroscope (IMU) to the simulated robot. Safe to run twice: it skips what is already done."""
import os
import re
import shutil
import sys
import time
import xml.etree.ElementTree as ET

src = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else '~/ros2_ws/src')
backup_dir = os.path.expanduser(sys.argv[2] if len(sys.argv) > 2 else '~/ros2_ws/project/backup_before_gyro')
desc = os.path.join(src, 'obs_avoid_description')
stamp = time.strftime('%Y%m%d_%H%M%S')
os.makedirs(backup_dir, exist_ok=True)
failed = False

IMU_BLOCK = '''
  <!-- gyroscope for the heading: a cheap MEMS gyro (like the MPU-6050) measures how fast the body turns -->
  <gazebo reference="base_link">
    <sensor name="imu" type="imu">
      <topic>/imu</topic>
      <gz_frame_id>base_link</gz_frame_id>
      <update_rate>50</update_rate>
      <always_on>true</always_on>
      <visualize>false</visualize>
      <imu>
        <angular_velocity>
          <z><noise type="gaussian"><mean>0.0</mean><stddev>0.002</stddev></noise></z>
        </angular_velocity>
      </imu>
    </sensor>
  </gazebo>
'''


def backup(path):
    shutil.copy2(path, os.path.join(backup_dir, os.path.basename(path) + '.' + stamp))


def edit(path, label, already, change, check_xml=False):
    """already: text that means 'done before'. change(text) -> new text or None if it cannot be done."""
    global failed
    if not os.path.isfile(path):
        print(f'{label}: FILE NOT FOUND: {path}')
        failed = True
        return
    text = open(path).read()
    if already in text:
        print(f'{label}: already done, skipped')
        return
    new = change(text)
    if new is None:
        print(f'{label}: could not find the place to edit, NOTHING CHANGED. Send me this file.')
        failed = True
        return
    backup(path)
    open(path, 'w').write(new)
    if check_xml:
        try:
            ET.parse(path)
        except ET.ParseError as err:
            shutil.copy2(os.path.join(backup_dir, os.path.basename(path) + '.' + stamp), path)
            print(f'{label}: the result was not valid XML ({err}); the original was restored')
            failed = True
            return
    print(f'{label}: done')


def add_imu(text):
    i = text.rfind('</robot>')
    return None if i < 0 else text[:i] + IMU_BLOCK + text[i:]


def add_bridge(text):
    m = re.search(r"^([ \t]*)'/sonar@[^\n]*\n", text, re.M)
    if not m:
        return None
    line = f"{m.group(1)}'/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',\n"
    return text[:m.end()] + line + text[m.end():]


def add_world_plugin(text):
    m = re.search(r'<world\b[^>]*>', text)
    if not m:
        return None
    plug = '\n    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>'
    return text[:m.end()] + plug + text[m.end():]


edit(os.path.join(desc, 'urdf', 'obs_avoid_gz.urdf'), 'robot file (URDF)', 'type="imu"', add_imu, check_xml=True)
edit(os.path.join(desc, 'launch', 'sim.launch.py'), 'ROS-Gazebo translator (launch file)', '/imu@', add_bridge)
edit(os.path.join(desc, 'worlds', 'obstacle_course.sdf'), 'world (IMU system)', 'gz-sim-imu-system', add_world_plugin,
     check_xml=True)
print('ALL GOOD' if not failed else 'SOMETHING FAILED, read the lines above')
sys.exit(1 if failed else 0)
