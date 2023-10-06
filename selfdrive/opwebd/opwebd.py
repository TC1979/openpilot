from flask import Flask
from openpilot.selfdrive.opwebd.components.home import route_home
from openpilot.selfdrive.opwebd.components.log import route_log
from openpilot.selfdrive.opwebd.components.params import route_params
from openpilot.selfdrive.opwebd.components.settings import route_settings
from openpilot.selfdrive.opwebd.components.update import route_update
from openpilot.selfdrive.opwebd.components.video import route_video
from openpilot.selfdrive.opwebd.components.gpx import route_gpx


app = Flask(__name__)

route_home(app)
route_log(app)
route_params(app)
route_settings(app)
route_update(app)
route_video(app)
route_gpx(app)


def main():
  app.run(host="0.0.0.0", port=5050)


if __name__ == '__main__':
  main()
