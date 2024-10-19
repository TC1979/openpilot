# PFEIFER - MAPD
import os
import subprocess
import urllib.request
import stat
import json
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper

VERSION = 'v1.9.0'
URL = f"https://github.com/pfeiferj/openpilot-mapd/releases/download/{VERSION}/mapd"
MAPD_PATH = '/data/media/0/osm/mapd'
VERSION_PATH = '/data/media/0/osm/mapd_version'

def get_gps_data(params):
  gps_data = params.get("LastGPSPosition")
  if gps_data:
    try:
      return json.loads(gps_data)
    except json.JSONDecodeError:
      print("Error decoding GPS data")
  return None

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

def mapd_thread(sm=None, pm=None):
  rk = Ratekeeper(0.05, print_delay_threshold=None)
  mem_params = Params("/dev/shm/params")

  while True:
    try:
      if not os.path.exists(MAPD_PATH) or not os.path.exists(VERSION_PATH):
        download()
        continue

      with open(VERSION_PATH) as f:
        content = f.read()
        if content != VERSION:
          download()
          continue

      gps_info = get_gps_data(mem_params)

      process = subprocess.Popen(cmd)
      process.wait()
    except Exception as e:
      print(f"Error in mapd_thread: {e}")

    rk.keep_time()


def main(sm=None, pm=None):
  mapd_thread(sm, pm)

if __name__ == "__main__":
  main()
