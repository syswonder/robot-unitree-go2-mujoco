#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Operator-started courtyard choreography, actual Atlas-discovered Robonix RPCs."""
import argparse
import json
import math
import concurrent.futures
from pathlib import Path
import time
from datetime import datetime
from test_robonix import connect
from robonix_api import ATLAS
from go2_sim.bridge import request
import chassis_pb2
import go2_sim_action_pb2
import lifecycle_pb2

ROOT=Path(__file__).resolve().parents[1]

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--auto',action='store_true')
    parser.add_argument('--check',action='store_true')
    parser.add_argument('--resume-after-jump',action='store_true',help='Rehearse C/D/F/E from current pose; not full-route acceptance')
    parser.add_argument('--live',action='store_true',help='Large, stationary terminal readout')
    args=parser.parse_args()
    state=request('/state')
    if state.get('scene')!='courtyard':
        raise RuntimeError('Start Native simulation with --scene courtyard')
    providers=ATLAS.query_primitives()
    print('ROBONIX x GO2 | COURTYARD DEMO',flush=True)
    print('Atlas -> Primitive RPC -> torque control -> MuJoCo',flush=True)
    print('生活园区 / 平地动作 + 低栏跳跃 + 组合行走',flush=True)
    print('这是预编排仿真演示，不是自主导航或实机特技。',flush=True)
    print('Primitives:',[(p.id,p.state.name) for p in providers],flush=True)
    me,mc,move=connect('robonix/primitive/chassis/move',
        chassis_pb2.ExecuteMoveCommand_Request,chassis_pb2.ExecuteMoveCommand_Response)
    ae,ac,action=connect('robonix/primitive/chassis/action',
        go2_sim_action_pb2.PerformAction_Request,go2_sim_action_pb2.PerformAction_Response)
    de,dc,driver=connect('robonix/primitive/chassis/driver',
        lifecycle_pb2.Driver_Request,lifecycle_pb2.Driver_Response)
    report={'transport':'Atlas-discovered gRPC','scope':'C/D/F/E rehearsal' if args.resume_after_jump else 'full route','complete':False,'steps':[]}
    pool=concurrent.futures.ThreadPoolExecutor(max_workers=1)
    telemetry={}
    def screen(label,command,status='RUNNING'):
        if not args.live:return
        s=request('/state');p=s['position'];v=s['linear_velocity']
        a=s.get('action',{})
        lines=['ROBONIX x GO2','COURTYARD / 园区演示',
            '4 ACTIVE primitives',
            f'{sum(len(p.capabilities) for p in providers)} registered capabilities',
            'Atlas -> Primitive RPC',
            ' -> 关节力矩 -> MuJoCo',
            '-----------------------------',label,status,
            '-----------------------------',command,
            '实时反馈',f'位置 X {p[0]:+.2f} m',f'位置 Y {p[1]:+.2f} m',
            f'前进速度 {v[0]:+.2f} m/s',
            f'航向 {math.degrees(s["yaw"]):+.1f} deg',
            f'已完成 {len(report["steps"])} 个步骤',
            f'动作 {a.get("name") or "walking"}',
            f'腾空 {a.get("airborne_s",0):.2f} s',
            '基础行走 / 低栏跳跃',
            '转向 / 侧移 / 蹲起',
            '楼梯 / 坡道 / 绕桩',
            '后空翻 / 倒立行走',
            '预编排仿真演示','Ctrl+C 停止']
        print('\033[H\033[2J'+'\n'.join(lines),flush=True)
    def call(label,command,method,req,timeout):
        telemetry.clear()
        telemetry.update(min_upright=1., max_height_m=0., samples=0)
        future=pool.submit(method,req,timeout=timeout)
        while not future.done():
            s=request('/state')
            telemetry['min_upright']=min(telemetry['min_upright'],s['upright'])
            telemetry['max_height_m']=max(telemetry['max_height_m'],s['position'][2])
            telemetry['samples']+=1
            screen(label,command)
            time.sleep(.15)
        return future.result()
    def invoke(label,**cmd):
        print('\n'+label+'\nRPC chassis/move '+json.dumps(cmd),flush=True)
        r=json.loads(call(label,'chassis/move',move,chassis_pb2.ExecuteMoveCommand_Request(
            command=chassis_pb2.MoveCommand(**cmd)),35).status.data)
        report['steps'].append({'label':label,'command':cmd,'result':r,'telemetry':dict(telemetry)})
        print('RESULT',r['status'],flush=True)
        if r['status']!='done':raise RuntimeError(str(r))
    def trick(label,name):
        print('\n'+label+'\nRPC chassis/action '+name,flush=True)
        r=call(label,'chassis/action: '+name,action,go2_sim_action_pb2.PerformAction_Request(name=name),55)
        metrics=json.loads(r.detail)
        report['steps'].append({'label':label,'action':name,'result':metrics})
        print('RESULT',r.status,'腾空',metrics.get('airborne_s'),'s',flush=True)
        if not r.success:raise RuntimeError(r.detail)
        if name=='jump' and not metrics.get('crossed_hurdle'):
            raise RuntimeError('Jump completed, but obstacle clearance did NOT pass: '+r.detail)
        time.sleep(1)
    def face(degrees,label):
        yaw=request('/state')['yaw']
        error=math.atan2(math.sin(math.radians(degrees)-yaw),math.cos(math.radians(degrees)-yaw))
        if abs(error)>.04:invoke(label,rotate_deg=math.degrees(error))
    def goto(x,y,label):
        # Measured waypoint choreography; no planner, pose writes or teleportation.
        for _ in range(4):
            s=request('/state');dx=x-s['position'][0];dy=y-s['position'][1]
            distance=math.hypot(dx,dy)
            if distance<.18:return
            face(math.degrees(math.atan2(dy,dx)),label+' / 对齐')
            invoke(label+' / 行走',forward_m=min(distance,4.5),linear_x=.40)
        s=request('/state')
        if math.hypot(x-s['position'][0],y-s['position'][1])>.20:
            raise RuntimeError('Waypoint did not converge: '+label)
    try:
        if args.check:return
        screen('准备演示','按 Enter 开始','READY')
        if not args.auto:input('\n开始屏幕录制后，按 Enter 执行完整流程；Ctrl+C 停止。')
        if not args.resume_after_jump:
            request('/command',{'reset':True})
            request('/command',{'view':'overview'})
            time.sleep(4)
            request('/command',{'view':'follow'})
            invoke('A区中心 / 左转一周',rotate_deg=360.)
            invoke('A区中心 / 右转一周',rotate_deg=-360.)
            invoke('A区 / 左侧移',linear_y=.25,duration_sec=2.)
            invoke('A区 / 右侧移',linear_y=-.25,duration_sec=2.)
            invoke('A区 / 后退',linear_x=-.3,duration_sec=2.)
            goto(0.,0.,'A区 / 返回中心')
            goto(2.7,0.,'进入B区')
            face(0.,'B区 / 对齐低栏')
            x=request('/state')['position'][0]
            invoke('B区 / 前进到起跳位置',forward_m=3.73-x)
            invoke('B区 / 停稳',duration_sec=2.)
            trick('B区中心 / 腾空跨栏','jump')
        goto(5.5,0.,'B区 / 落地继续前进')
        for i,(x,y) in enumerate(((5.6,3.2),(4.6,3.25),(4.6,4.25),(3.35,4.7),
                                 (3.05,5.3),(3.05,6.6),(4.65,6.75))):
            goto(x,y,f'C区 / 绕桩路线 {i+1}')
        goto(1.4,6.9,'前往D区通道')
        goto(1.4,5.,'D区 / 楼梯中心线入口')
        face(180.,'D区 / 对齐楼梯')
        invoke('D区 / 上楼梯到平台',forward_m=2.35,linear_x=.25)
        invoke('D区 / 越过平台并下楼梯',forward_m=2.35,linear_x=.25)
        goto(-3.6,7.,'前往F区通道')
        goto(-5.,7.,'F区 / 坡道中心线入口')
        face(-90.,'F区 / 对齐坡道')
        invoke('F区 / 上坡-越顶-下坡',forward_m=3.5,linear_x=.25)
        goto(-5.,0.,'进入E区表演中心')
        face(0.,'E区 / 面向观众')
        trick('E区中心 / 深蹲起立','crouch')
        trick('E区中心 / 鞠躬','bow')
        trick('E区中心 / 左右摇摆','sway')
        trick('E区中心 / 伸展','stretch')
        trick('E区中心 / 节律舞蹈','dance')
        trick('E区中心 / 后空翻','backflip')
        goto(-5.3,0.,'E区 / 倒立行走起点')
        face(0.,'E区 / 倒立方向')
        trick('E区 / 倒立-前进-恢复四足','handstand_walk')
        goto(-5.,0.,'E区 / 返回中心谢幕')
        trick('E区 / 鞠躬谢幕','bow')
        invoke('结束 / 稳定停止',duration_sec=3.)
        request('/command',{'view':'overview'})
        time.sleep(3)
        report['complete']=True
        screen('全部动作完成','Robonix RPC','COMPLETE')
        print('\n完整流程结束：动作由 Robonix RPC 编排执行。',flush=True)
    finally:
        interrupted=not args.check and not report['complete']
        if interrupted:
            driver(lifecycle_pb2.Driver_Request(command=2),timeout=5)
        if not args.check:
            request('/command',{'stop':True})
        pool.shutdown(wait=True)
        if interrupted:
            driver(lifecycle_pb2.Driver_Request(command=1),timeout=5)
        if not args.check:
            request('/command',{'stop':True})
            out=ROOT/'.runtime'/('courtyard-showcase-'+datetime.now().strftime('%Y%m%dT%H%M%S')+'.json')
            out.write_text(json.dumps(report,ensure_ascii=False,indent=2))
            print('运行记录:',out,flush=True)
        for handle in (me,mc,ae,ac,de,dc):handle.close()

if __name__=='__main__':main()
