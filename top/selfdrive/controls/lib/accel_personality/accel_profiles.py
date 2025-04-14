"""
Copyright (c) 2021-, rav4kumar, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

# Acceleration profile for maximum allowed acceleration
MAX_ACCEL_ECO     = [2.0, 1.7, 1.1,  .95, .75, .70, .65, .45, .32, .20, .085]
MAX_ACCEL_NORMAL  = [2.2, 1.9, 1.4, 1.22, .95, .83, .71, .54, .45, .38, .15]
MAX_ACCEL_SPORT   = [3.0, 2.4, 2.0, 1.45, 1.2, 1.0, .92, .75, .63, .55, .25]
# MAX_ACCEL kmh =   [0.,  3,   10,   20,   30,  40,  53,  72,  90,  107, 150]

# Acceleration profile for minimum (braking) acceleration
MIN_ACCEL_ECO     = [-1.0, -1.0, -1.0, -1.0, -1.0]
MIN_ACCEL_NORMAL  = [-1.1, -1.1, -1.1, -1.1, -1.1]
MIN_ACCEL_SPORT   = [-1.4, -1.4, -1.4, -1.4, -1.4]
MIN_ACCEL_STOCK   = [-1.2, -1.2, -1.2, -1.2, -1.2]

# Speed breakpoints for interpolation
MAX_ACCEL_BREAKPOINTS = [0., 1., 3., 6., 8., 11., 16., 20., 25., 30., 55.]
MIN_ACCEL_BREAKPOINTS = [0., 0.3, 1., 27, 40]
