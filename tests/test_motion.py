# SPDX-License-Identifier: Apache-2.0
import unittest
from go2_sim.motion import distance_velocity


class DistanceVelocity(unittest.TestCase):
    def test_cruise_default_and_limit(self):
        self.assertEqual(distance_velocity(2.), .3)
        self.assertEqual(distance_velocity(2., .25), .25)
        self.assertEqual(distance_velocity(2., 8.), .5)

    def test_explicit_low_speed_is_not_raised(self):
        self.assertEqual(distance_velocity(2., .1), .1)
        self.assertEqual(distance_velocity(.05, .1), .1)

    def test_reverse_and_deadband(self):
        self.assertEqual(distance_velocity(-.05, .25), -.18)
        self.assertEqual(distance_velocity(-2., .25), -.25)
