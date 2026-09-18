# SPDX-License-Identifier: Apache-2.0
"""Simulation-only joint-torque choreography, not Unitree's proprietary tricks.

No root-pose assignment, external forces, gravity changes or asset edits.
Walking delegates unchanged to the existing pinned RL controller.
"""
import numpy as np
import mujoco
import importlib.util
from pathlib import Path
from .controller import Go2Controller

ACTION_DURATIONS = {'crouch': 4., 'bow': 4., 'dance': 7., 'sway':6., 'stretch':6., 'jump': 4., 'handstand_walk':27., 'backflip':10.}
ASSETS = Path(__file__).resolve().parents[1]/'.runtime/assets'

def available_actions():
    """Ordinary walking remains usable without the optional ONNX dependencies."""
    result = list(ACTION_DURATIONS)
    ort = importlib.util.find_spec('onnxruntime') is not None
    optional = {'handstand_walk': ('quad2hand/go2_actor.onnx', 'quad2hand/go2_estimator.onnx'),
                'backflip': ('backflip/policy.onnx',)}
    for name, files in optional.items():
        if not ort or not all((ASSETS / path).is_file() for path in files):
            result.remove(name)
    return result

JUMP_PARAMETERS = (1.1830620111643995, 1.164493942887515, .8160593238428335,
                   .26794865082447106, .2609987600409997, -.0574244296546308,
                   -.02407907678020789)

def smooth(x):
    x = np.clip(x, 0., 1.)
    return x*x*(3.-2.*x)

class ActionController(Go2Controller):
    def reset(self):
        super().reset()
        self.action = {'id': '', 'name': '', 'status': 'idle'}
        self.active = False

    def start_action(self, name, token):
        if name not in available_actions():
            raise ValueError('Unsupported action: '+name)
        if name=='handstand_walk':
            if not hasattr(self,'hand_policy'):
                from .quad2hand import Quad2Hand
                self.hand_policy=Quad2Hand(self.model,self.data,ASSETS)
            self.hand_policy.reset()
        if name=='backflip':
            if not hasattr(self,'flip_policy'):
                from .backflip import Backflip
                self.flip_policy=Backflip(self.model,self.data,ASSETS)
            self.flip_policy.reset()
        self.action = {'id':token, 'name':name, 'status':'running', 'elapsed_s':0.,
                       'max_height_m':float(self.data.xpos[self.bid,2]),
                       'airborne_s':0., 'obstacle_contacts':0, 'nonfoot_ground_contacts':0}
        self.started = float(self.data.time)
        self.origin = self.data.xpos[self.bid].copy()
        self.startq = self.data.qpos[self.qadr].copy()
        self.active = True
        self.air_run = 0.
        self.action['handstand_s']=0.
        self.action['pitch_rotation_rad']=0.
        self.foot_geoms = {self.model.geom(n).id for n in ('FR','FL','RR','RL')}
        self.floor_geom = self.model.geom('floor').id
        self.action['initial_foot_x'] = [float(self.data.xpos[self.model.body(n+'_foot').id,0])
                                          for n in ('FR','FL','RR','RL')]

    def finish(self, status):
        self.action['status'] = status
        self.action['displacement_m'] = (self.data.xpos[self.bid]-self.origin).tolist()
        self.action['upright'] = float(self.data.xmat[self.bid].reshape(3,3)[2,2])
        feet=[float(self.data.xpos[self.model.body(n+'_foot').id,0]) for n in ('FR','FL','RR','RL')]
        self.action['final_foot_x']=feet
        hurdle=mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_GEOM,'jump_hurdle')
        if hurdle>=0:
            x=float(self.model.geom_pos[hurdle,0]);half=float(self.model.geom_size[hurdle,0])
            self.action['crossed_hurdle']=(max(self.action['initial_foot_x'])<x-half
                and min(feet)>x+half and self.action['obstacle_contacts']==0
                and self.action['airborne_s']>.05)
        self.active = False
        self.actions[:] = 0.
        self.steps = 0
        self.yaw_target = self.state()['yaw']

    def cancel_action(self):
        if self.active:
            self.finish('cancelled')

    def step(self, command):
        if not self.active:
            return super().step(command)
        t = float(self.data.time)-self.started
        name = self.action['name']
        target = self.default.copy()
        kp, kd = 65., 2.
        # Settle through real torques; no reset between actions.
        learned = name in ('handstand_walk','backflip')
        if name=='backflip':
            self.flip_policy.step(t)
        elif name=='handstand_walk':
            mode=1 if 3<=t<20 else 0
            command=[.20 if 9<t<16 else 0.,0.,0.]
            self.hand_policy.step(mode,command)
            rot=self.data.xmat[self.bid].reshape(3,3)
            rear_z=min(self.data.xpos[self.model.body(n+'_foot').id,2] for n in ('RR','RL'))
            if -rot[2,0]>.90 and rear_z>.4:
                self.action['handstand_s']+=self.model.opt.timestep
            if 9<=t<9.01:self.hand_origin=self.data.xpos[self.bid].copy()
            if 16<=t<16.01 and hasattr(self,'hand_origin'):
                self.action['handstand_travel_m']=float(np.linalg.norm((self.data.xpos[self.bid]-self.hand_origin)[:2]))
        elif t < 1.:
            target = self.startq*(1-smooth(t/.6))+self.default*smooth(t/.6)
        elif name == 'jump':
            t0 = t-1.
            crouch, lean, extension, thrust_s, tuck_s, bias, prelean = JUMP_PARAMETERS
            squat = np.tile([0.,crouch+prelean,-2*crouch],4)
            if t0 < .5:
                target=self.default*(1-smooth(t0/.5))+squat*smooth(t0/.5)
            elif t0 < .7: target=squat
            elif t0 < .7+thrust_s:
                target=np.tile([0.,extension/2+lean,-extension],4)
                target[[1,4]]+=bias
                target[[7,10]]-=bias
                kp,kd=90.,1.5
            elif t0 < .7+thrust_s+tuck_s:
                target=np.tile([0.,1.05,-2.1],4)
        elif name in ('crouch','bow'):
            blend=smooth((t-1.)/.6)*(1-smooth((t-2.2)/.6))
            pose=np.tile([0.,1.27,-2.54],4) if name=='crouch' else self.default.copy()
            if name=='bow':
                pose[[1,4]]+=.34
                pose[[2,5]]-=.68
                pose[[7,10]]-=.08
                pose[[8,11]]+=.16
            target=self.default*(1-blend)+pose*blend
        elif name=='dance':
            envelope=smooth((t-1.)/.6)*(1-smooth((t-5.8)/.6))
            phase=(t-1.)*2*np.pi*.65
            target[::3]+=.20*np.sin(phase)*envelope
            target[1::3]+=.18*np.sin(phase*2)*envelope
            target[2::3]-=.36*np.sin(phase*2)*envelope
        elif name in ('sway','stretch'):
            envelope=smooth((t-1.)/.6)*(1-smooth((t-4.8)/.6))
            wave=np.sin((t-1.)*2*np.pi*.5)*envelope
            if name=='sway':
                target[::3]+=.26*wave
            else:
                target[[1,4]]+=.10*wave
                target[[2,5]]-=.20*wave
                target[[7,10]]-=.10*wave
                target[[8,11]]+=.20*wave
        if not learned:
            torque=kp*(target-self.data.qpos[self.qadr])-kd*self.data.qvel[self.vadr]
            self.data.ctrl[self.aids]=np.clip(torque,-self.limits,self.limits)
            mujoco.mj_step(self.model,self.data)
        if not np.isfinite(self.data.qpos).all() or any(w.number for w in self.data.warning):
            raise RuntimeError('MuJoCo numerical failure during action')
        self.action['elapsed_s']=t
        self.action['pitch_rotation_rad']+=float(self.data.qvel[4])*self.model.opt.timestep
        self.action['max_height_m']=max(self.action['max_height_m'],float(self.data.xpos[self.bid,2]))
        ground=False
        for contact in self.data.contact:
            ids=(contact.geom1,contact.geom2)
            if self.floor_geom in ids:
                ground=True
                if not self.foot_geoms.intersection(ids):
                    self.action['nonfoot_ground_contacts']+=1
            if any(self.model.geom(i).name=='jump_hurdle' for i in ids):
                self.action['obstacle_contacts']+=1
        self.air_run=0. if ground else self.air_run+self.model.opt.timestep
        self.action['airborne_s']=max(self.action['airborne_s'],self.air_run)
        if t >= ACTION_DURATIONS[name]:
            up=float(self.data.xmat[self.bid].reshape(3,3)[2,2])
            good=up>.85 and self.data.xpos[self.bid,2]>.2 and self.action['nonfoot_ground_contacts']==0
            if name=='backflip':good &= abs(self.action['pitch_rotation_rad'])>5.8
            if name=='handstand_walk':good &= self.action['handstand_s']>4 and self.action.get('handstand_travel_m',0)>.4
            self.finish('done' if good else 'failed')
