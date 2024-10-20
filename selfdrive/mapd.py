# PFEIFER - MAPD
import os
import subprocess
import urllib.request
import math
import json
import cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
import stat

VERSION = 'v1.9.0'
URL = f"https://github.com/pfeiferj/openpilot-mapd/releases/download/{VERSION}/mapd"
MAPD_PATH = '/data/media/0/osm/mapd'
VERSION_PATH = '/data/media/0/osm/mapd_version'

def download():
  mapd_dir = os.path.dirname(MAPD_PATH)
  if not os.path.exists(mapd_dir):
    os.makedirs(mapd_dir)
  with urllib.request.urlopen(URL) as f:
    with open(MAPD_PATH, 'wb') as output:
      output.write(f.read())
      os.fsync(output)
      current_permissions = stat.S_IMODE(os.lstat(MAPD_PATH).st_mode) # <-- preserve permissions
      os.chmod(MAPD_PATH, current_permissions | stat.S_IEXEC) # <-- preserve permissions
    with open(VERSION_PATH, 'w') as output:
      output.write(VERSION)
      os.fsync(output)

def get_gps_data(params):
  gps_data = params.get("LastGPSPosition")
  if gps_data:
    try:
      return json.loads(gps_data)
    except json.JSONDecodeError:
      print("Error decoding GPS data")
  return None

def convert_live_pose_to_location_kalman(live_pose):
  location_kalman = {
    "positionGeodetic": {
      "lat": live_pose.orientationNED.x,
      "lon": live_pose.orientationNED.y,
      "alt": live_pose.orientationNED.z
    },
    "calibratedOrientationNED": {
      "x": live_pose.orientationNED.x,
      "y": live_pose.orientationNED.y,
      "z": live_pose.orientationNED.z
    },
    "velocityNED": {
      "x": live_pose.velocityDevice.x,
      "y": live_pose.velocityDevice.y,
      "z": live_pose.velocityDevice.z
    }
  }
  return location_kalman

def mapd_thread(sm=None, pm=None):
  rk = Ratekeeper(0.05, print_delay_threshold=None)
  mem_params = Params("/dev/shm/params")

  while True:
    try:
      if sm is not None:
        sm.update()
        if sm.updated['livePose']:
          live_pose = sm['livePose']
          location_kalman = convert_live_pose_to_location_kalman(live_pose)
          mem_params.put("LastGPSPosition", json.dumps({
            "latitude": float(location_kalman["positionGeodetic"]["lat"]),
            "longitude": float(location_kalman["positionGeodetic"]["lon"]),
            "bearing": float(math.degrees(location_kalman["calibratedOrientationNED"]["z"]))
          }))

      gps_info = get_gps_data(mem_params)

      if not os.path.exists(MAPD_PATH) or not os.path.exists(VERSION_PATH):
        download()
        continue

      with open(VERSION_PATH) as f:
        content = f.read()
        if content != VERSION:
          download()
          continue

      cmd = [MAPD_PATH]
      if gps_info:
        cmd.extend([
          "--latitude", str(gps_info["latitude"]),
          "--longitude", str(gps_info["longitude"]),
          "--bearing", str(gps_info["bearing"])
        ])

      process = subprocess.Popen(cmd)
      process.wait()
    except Exception as e:
      print(f"Error in mapd_thread: {e}")

    rk.keep_time()


def main(sm=None, pm=None):
  if sm is None:
    sm = messaging.SubMaster(['livePose'])
  mapd_thread(sm, pm)

if __name__ == "__main__":
  main()
