#!/usr/bin/env python3

from openpilot.common.conversions import Conversions as CV

class AlternativeDrivingPersonalityController:
    def __init__(self):
        # try except to avoid issue in longitudinal testing
        try:
            self._speed = 11 * CV.KPH_TO_MS
        except Exception:
            self._speed = 0

        self._mode = 1
        self._active = False

    def update(self, v_ego):
        self._active = self._speed > 0 and v_ego < self._speed

    def get_personality(self, personality):
        return self._mode if self._active else personality

    def is_active(self):
        return self._active
