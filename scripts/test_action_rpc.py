#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Verify action/move mutual exclusion and lifecycle cancellation via real RPC."""
import concurrent.futures
import json
import time
from pathlib import Path
from test_robonix import connect
from go2_sim.bridge import request
import chassis_pb2
import lifecycle_pb2
import go2_sim_action_pb2

def main():
    ae,ac,action=connect('robonix/primitive/chassis/action',
        go2_sim_action_pb2.PerformAction_Request,go2_sim_action_pb2.PerformAction_Response)
    me,mc,move=connect('robonix/primitive/chassis/move',
        chassis_pb2.ExecuteMoveCommand_Request,chassis_pb2.ExecuteMoveCommand_Response)
    de,dc,driver=connect('robonix/primitive/chassis/driver',
        lifecycle_pb2.Driver_Request,lifecycle_pb2.Driver_Response)
    report={}
    try:
        request('/command',{'reset':True})
        time.sleep(2.)
        invalid=action(go2_sim_action_pb2.PerformAction_Request(name='unknown-trick'),timeout=5)
        assert not invalid.success and invalid.status=='error'
        report['unsupported_action']=invalid.status
        with concurrent.futures.ThreadPoolExecutor() as pool:
            future=pool.submit(action,go2_sim_action_pb2.PerformAction_Request(name='dance'),timeout=20)
            for _ in range(40):
                if request('/state').get('action',{}).get('status')=='running':break
                time.sleep(.1)
            else:raise AssertionError('Action never started')
            busy=move(chassis_pb2.ExecuteMoveCommand_Request(
                command=chassis_pb2.MoveCommand(linear_x=.2,duration_sec=1)),timeout=5)
            report['concurrent_move']=json.loads(busy.status.data)['status']
            assert report['concurrent_move']=='busy'
            assert driver(lifecycle_pb2.Driver_Request(command=2),timeout=5).ok
            result=future.result(timeout=5)
            assert result.status=='cancelled' and not result.success
            report['lifecycle_cancel']=result.status
        time.sleep(1.)
        assert request('/state')['action']['status']=='cancelled'
        assert driver(lifecycle_pb2.Driver_Request(command=1),timeout=5).ok
        result=action(go2_sim_action_pb2.PerformAction_Request(name='bow'),timeout=20)
        assert result.success
        report['reactivated_action']=result.status
        report['passed']=True
        out=Path(__file__).resolve().parents[1]/'.runtime/action-rpc-acceptance.json'
        out.write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report,indent=2))
    finally:
        request('/command',{'stop':True})
        driver(lifecycle_pb2.Driver_Request(command=1),timeout=5)
        for handle in (ae,ac,me,mc,de,dc):handle.close()

if __name__=='__main__':main()
