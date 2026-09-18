#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Measure torque-only actions and the fixed courtyard hurdle, no ROS/hardware."""
import json
from pathlib import Path
import mujoco
from go2_sim.scene import load_scene
from go2_sim.actions import ActionController, ACTION_DURATIONS

ROOT=Path(__file__).resolve().parents[1]
def main():
    report=[]
    for name in ACTION_DURATIONS:
        model=load_scene(ROOT/'.runtime/assets',scene='courtyard')
        data=mujoco.MjData(model)
        controller=ActionController(model,data,ROOT/'.runtime/assets')
        for _ in range(400): controller.step([0,0,0])
        if name=='jump':
            for _ in range(4000):
                controller.step([.3,0,0])
                if data.xpos[controller.bid,0]>=3.65: break
            for _ in range(400): controller.step([0,0,0])
        before=controller.state()
        controller.start_action(name,name)
        contact_first=None
        while controller.active:
            controller.step([0,0,0])
            if controller.action['obstacle_contacts'] and contact_first is None:
                contact_first={'time':controller.action['elapsed_s'], 'position':controller.state()['position']}
        result=dict(controller.action)
        result['start_position']=before['position']
        result['first_obstacle_contact']=contact_first
        result['end_position']=controller.state()['position']
        result['foot_x_after']=[float(data.xpos[model.body(n+'_foot').id,0]) for n in ('FR','FL','RR','RL')]
        result['passed']=result['status']=='done' and result['nonfoot_ground_contacts']==0
        if name=='jump':
            result['passed'] &= result['crossed_hurdle']
        report.append(result)
        print(json.dumps(result),flush=True)
    (ROOT/'.runtime/actions-acceptance.json').write_text(json.dumps(report,indent=2))
    return all(r['passed'] for r in report)
if __name__=='__main__':
    raise SystemExit(0 if main() else 1)
