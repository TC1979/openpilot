"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

from cereal import messaging
from openpilot.top.selfdrive.controls.lib.accel_personality.accel_controller import AccelController

class LongitudinalPlannerTOP:
  def __init__(self):
    self.accel_controller = AccelController()

  def update(self, sm: messaging.SubMaster) -> None:
    self.accel_controller.update()

  def publish_longitudinal_plan_top(self, sm: messaging.SubMaster, pm: messaging.PubMaster) -> None:
    plan_top_send = messaging.new_message('longitudinalPlanTOP')

    plan_top_send.valid = sm.all_checks(service_list=['carState', 'controlsState'])

    pm.send('longitudinalPlanTOP', plan_top_send)
