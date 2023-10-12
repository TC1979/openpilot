# PFEIFER - OPWEBGO
import os
import subprocess
import urllib.request
from openpilot.common.realtime import Ratekeeper
import stat

VERSION = 'v1.0.1'
URL = f"https://github.com/pfeiferj/opweb/releases/download/{VERSION}/opwebd"
OPWEBD_PATH = '/data/openpilot/opwebd'
VERSION_PATH = '/data/openpilot/opwebd_version'

def download():
  with urllib.request.urlopen(URL) as f:
    with open(OPWEBD_PATH, 'wb') as output:
      output.write(f.read())
      os.fsync(output)
      os.chmod(OPWEBD_PATH, stat.S_IEXEC)
    with open(VERSION_PATH, 'w') as output:
      output.write(VERSION)
      os.fsync(output)

def opwebd_thread(sm=None, pm=None):
  rk = Ratekeeper(0.05, print_delay_threshold=None)

  while True:
    try:
      if not os.path.exists(OPWEBD_PATH):
        download()
        continue
      if not os.path.exists(VERSION_PATH):
        download()
        continue
      with open(VERSION_PATH) as f:
        content = f.read()
        if content != VERSION:
          download()
          continue

      process = subprocess.Popen('/data/openpilot/opwebd', stdout=subprocess.PIPE)
      process.wait()
    except Exception as e:
      print(e)

    rk.keep_time()


def main(sm=None, pm=None):
  opwebd_thread(sm, pm)

if __name__ == "__main__":
  main()
