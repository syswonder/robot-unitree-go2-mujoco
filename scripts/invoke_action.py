#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Call one simulated action through Atlas/gRPC, independently of the courtyard."""
import argparse
import json
import go2_sim_action_pb2
import lifecycle_pb2
from go2_sim.bridge import request
from test_robonix import connect


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('name', nargs='?', help='Action from --list; no automatic reset or repositioning')
    parser.add_argument('--list', action='store_true', help='Read advertised simulator actions without moving')
    args = parser.parse_args()
    if args.list:
        print(json.dumps(request('/state').get('available_actions', []), indent=2))
        return 0
    if not args.name:
        parser.error('provide an action name or --list')
    ae, ac, action = connect('robonix/primitive/chassis/action',
        go2_sim_action_pb2.PerformAction_Request, go2_sim_action_pb2.PerformAction_Response)
    de, dc, driver = connect('robonix/primitive/chassis/driver',
        lifecycle_pb2.Driver_Request, lifecycle_pb2.Driver_Response)
    pending = None
    try:
        pending = action.future(go2_sim_action_pb2.PerformAction_Request(name=args.name), timeout=55.)
        result = pending.result()
        print(json.dumps({'success': result.success, 'status': result.status,
                         'detail': json.loads(result.detail)}, indent=2))
        return 0 if result.success else 1
    finally:
        # Normal completion already releases the native lease. On interruption,
        # request the existing lifecycle stop; do not silently reactivate it.
        try:
            if pending is not None and not pending.done():
                driver(lifecycle_pb2.Driver_Request(command=2), timeout=5.)
                pending.cancel()
        finally:
            for handle in (ae, ac, de, dc):
                handle.close()


if __name__ == '__main__':
    raise SystemExit(main())
