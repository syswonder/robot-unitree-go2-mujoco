# SPDX-License-Identifier: MulanPSL-2.0
"""Regression fixtures for the actual Navigation FastMCP wire response."""
import asyncio
import json
import sys
import unittest
from types import SimpleNamespace
from unittest.mock import patch

from explore_skill.controller import ExploreController


class McpWireTest(unittest.TestCase):
    def test_actual_wrapped_speed_response(self):
        """Decode the live Any-result envelope before validating speed availability."""
        payload = dict(available=True, max_linear_speed_mps=.25,
            default_percentage=80., min_percentage=20., step_percentage=20.,
            effective_percentage=80., effective_linear_speed_mps=.2,
            scope='session', run_id='', detail='navigation speed limit is available')
        self.assertEqual(self.read_wire({'result': payload}, payload), payload)

    def test_navigation_and_cancel_wrappers_are_decoded(self):
        """The same envelope appears on accepted and rejected navigation results."""
        for payload in ({'accepted': True, 'run_id': 'owned'}, {'accepted': False},
                        {'known': True, 'state': 'CANCELED'}):
            with self.subTest(payload=payload):
                self.assertEqual(self.read_wire({'result': payload}, payload), payload)

    def test_flat_and_text_only_responses_remain_supported(self):
        payload = {'available': False}
        self.assertEqual(self.read_wire(payload, payload), payload)
        self.assertEqual(self.read_wire(None, payload), payload)

    def read_wire(self, structured, text):
        """Replace only the network client while exercising production decoding."""
        result = SimpleNamespace(is_error=False, structured_content=structured,
                                 content=[SimpleNamespace(text=json.dumps(text))])
        class Client:
            def __init__(self, endpoint):
                self.endpoint = endpoint
            async def __aenter__(self):
                return self
            async def __aexit__(self, *args):
                return None
            async def call_tool(self, tool, args):
                return result
        controller = ExploreController(map_topic='/map', nav_navigate_endpoint='nav',
                                       nav_status_endpoint='nav', nav_cancel_endpoint='nav')
        with patch.dict(sys.modules, fastmcp=SimpleNamespace(Client=Client)):
            return asyncio.run(controller._mcp_call('get_speed_limit', {}))


if __name__ == '__main__':
    unittest.main()
