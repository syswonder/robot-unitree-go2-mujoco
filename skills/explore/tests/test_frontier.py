# SPDX-License-Identifier: MulanPSL-2.0
"""Raster regressions for furniture centroids, disconnected rooms and temporary failures."""
import math
import unittest
from types import SimpleNamespace

import numpy as np

from explore_skill.frontier import GridView, footprint_free, grid_distances, pick_target
from explore_skill.configuration import parse_config


def grid(data, origin_x=0.0):
    """Create a five-centimetre synthetic occupancy map without scene asset shortcuts."""
    return GridView(data, 0.05, origin_x, 0.0, data.shape[1], data.shape[0])


class FrontierTest(unittest.TestCase):
    def corridor(self):
        """Known corridor has an unknown continuation at each end."""
        data = np.full((60, 140), 100, dtype=np.int8)
        data[5:55, 5:135] = 0
        data[5:55, :5] = -1
        data[5:55, 135:] = -1
        return grid(data)

    def test_centroid_inside_sofa_is_not_a_goal(self):
        """A frontier ring averages into an unknown furniture interior; approach stays free."""
        data = np.zeros((100, 100), dtype=np.int8)
        data[:5, :] = data[-5:, :] = data[:, :5] = data[:, -5:] = 100
        data[35:65, 35:65] = -1
        gv = grid(data)
        target = pick_target(gv, (1.0, 2.5))
        self.assertIsNotNone(target)
        cx, cy = gv.world_to_cell(*target.centroid_xy)
        self.assertEqual(gv.data[cy, cx], 0)
        self.assertTrue(footprint_free(gv, math.hypot(.35, .2))[cy, cx])
        self.assertFalse(35 <= cx < 65 and 35 <= cy < 65)

    def test_disconnected_free_room_is_not_reachable(self):
        """An attractive unknown edge across a wall cannot supply a target."""
        data = np.full((60, 120), 100, dtype=np.int8)
        data[5:55, 5:50] = 0
        data[5:55, 60:110] = 0
        data[5:55, 110:] = -1
        self.assertIsNone(pick_target(grid(data), (1.0, 1.5)))

    def test_failed_neighborhood_uses_world_coordinates_and_expires(self):
        """Failure skips one end, survives map-origin changes and expires deterministically."""
        gv = self.corridor()
        robot = (3.5, 1.5)
        first = pick_target(gv, robot)
        failed = [(*first.centroid_xy, 20.0)]
        second = pick_target(gv, robot, failed_goals=failed, failed_goal_radius_m=2.5, now=10)
        self.assertIsNotNone(second)
        self.assertGreater(math.dist(first.centroid_xy, second.centroid_xy), 2.5)
        shifted = grid(np.pad(gv.data, ((0, 0), (20, 0)), constant_values=100), origin_x=-1.0)
        again = pick_target(shifted, robot, failed_goals=failed, failed_goal_radius_m=2.5, now=10)
        self.assertAlmostEqual(math.dist(again.centroid_xy, second.centroid_xy), 0)
        expired = pick_target(gv, robot, failed_goals=failed, failed_goal_radius_m=2.5, now=21)
        self.assertEqual(expired.centroid_xy, first.centroid_xy)

    def test_narrow_gap_cannot_connect_footprint(self):
        """A point-connected gap narrower than the robot cannot connect safe candidates."""
        gv = self.corridor()
        gv.data[:, 60:63] = 100
        gv.data[27:33, 60:63] = 0
        safe = footprint_free(gv, math.hypot(.35, .2))
        reachable = grid_distances(safe, [gv.world_to_cell(1.0, 1.5)])
        self.assertLess(reachable[30, 100], 0)

    def test_new_map_opens_a_real_doorway(self):
        """A 0.90 m observed doorway enables a next-room approach on recomputation."""
        data = np.full((100, 160), 100, dtype=np.int8)
        data[5:95, 5:150] = 0
        data[:, 78:82] = 100
        data[5:95, 150:] = -1
        self.assertIsNone(pick_target(grid(data), (3.0, 2.5)))
        data[41:59, 78:82] = 0
        target = pick_target(grid(data), (3.0, 2.5))
        self.assertIsNotNone(target)
        self.assertGreater(target.centroid_xy[0], 4.1)

    def test_signed_ros_values_and_negative_world_bounds(self):
        """Python signed ROS lists preserve unknown values and points outside the grid."""
        msg = SimpleNamespace(data=[-1, 0, 100, 0], info=SimpleNamespace(
            height=2, width=2, resolution=.05, origin=SimpleNamespace(position=SimpleNamespace(x=0,y=0))))
        gv = GridView.from_msg(msg)
        self.assertEqual(gv.data[0, 0], -1)
        self.assertEqual(gv.world_to_cell(-.01, -.01), (-1, -1))

    def test_invalid_config_rejected(self):
        """Reject unbounded clearance settings and request limits in package config."""
        for value in (0, -1, 0.2, float('nan'), float('inf'), True):
            with self.subTest(value=value), self.assertRaises(ValueError):
                parse_config({'robot_radius_m': value})
        with self.assertRaises(ValueError):
            parse_config({'timeout_s': 60})


if __name__ == '__main__':
    unittest.main()
