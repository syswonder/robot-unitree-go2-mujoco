# SPDX-License-Identifier: Apache-2.0
"""Small feedback laws shared by the simulated chassis and offline tests."""
import math


def distance_velocity(error, requested_speed=0.):
    """Avoid the policy deadband without exceeding an explicit cruise speed."""
    cruise = min(.5, abs(requested_speed)) if requested_speed else .3
    return math.copysign(min(cruise, max(.18, abs(error))), error)
