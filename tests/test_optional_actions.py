# SPDX-License-Identifier: Apache-2.0
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
from go2_sim.actions import available_actions


class OptionalActions(unittest.TestCase):
    def test_without_onnx(self):
        with patch('go2_sim.actions.importlib.util.find_spec', return_value=None):
            names = available_actions()
        self.assertIn('jump', names)
        self.assertNotIn('backflip', names)
        self.assertNotIn('handstand_walk', names)

    def test_assets_required(self):
        # Use repository-local temp storage, not another filesystem.
        root = Path(__file__).resolve().parents[1] / '.runtime'
        root.mkdir(exist_ok=True)
        with tempfile.TemporaryDirectory(dir=root) as folder:
            with patch('go2_sim.actions.ASSETS', Path(folder)), patch(
                    'go2_sim.actions.importlib.util.find_spec', return_value=object()):
                self.assertNotIn('backflip', available_actions())
                target = Path(folder) / 'backflip/policy.onnx'
                target.parent.mkdir()
                target.touch()
                self.assertIn('backflip', available_actions())
                self.assertNotIn('handstand_walk', available_actions())
