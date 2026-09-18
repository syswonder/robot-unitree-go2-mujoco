# SPDX-License-Identifier: MIT
# Observation/actuation semantics: Robot-Nav/GO2_backflip, PPO-backflip branch.
# See third_party/backflip-LICENSE and stunt-assets.lock.json.
"""Phase-conditioned public backflip policy; no changes to plant dynamics."""
from pathlib import Path
import numpy as np
import mujoco
import onnxruntime as ort

class Backflip:
    def __init__(self,model,data,assets):
        self.model,self.data=model,data
        names=[leg+'_'+joint+'_joint' for leg in ('FL','FR','RL','RR') for joint in ('hip','thigh','calf')]
        jids=[model.joint(n).id for n in names]
        self.qadr=model.jnt_qposadr[jids];self.vadr=model.jnt_dofadr[jids]
        self.aids=[model.actuator(n.removesuffix('_joint')).id for n in names]
        self.bid=model.body('base_link').id
        opts=ort.SessionOptions();opts.intra_op_num_threads=1;opts.inter_op_num_threads=1
        self.net=ort.InferenceSession(str(Path(assets)/'backflip/policy.onnx'),opts,providers=['CPUExecutionProvider'])
        self.default=np.array([0,.8,-1.5,0,.8,-1.5,0,1.,-1.5,0,1.,-1.5])
        self.reset()

    def reset(self):
        self.current=np.zeros(12);self.last=np.zeros(12);self.slew=np.zeros(12)
        self.applied=np.zeros(12);self.steps=0
        self.startq=self.data.qpos[self.qadr].copy()

    def step(self,t):
        m,d=self.model,self.data
        if t<2.:
            blend=np.clip(t/.8,0,1);blend=blend*blend*(3-2*blend)
            target=self.startq*(1-blend)+self.default*blend
            tau=40*(target-d.qpos[self.qadr])-d.qvel[self.vadr]
            d.ctrl[self.aids]=np.clip(tau,-23.4,23.4)
        else:
            if self.steps%4==0:
                phase=np.pi*min(t-2.,2.)/2
                f=[np.sin(phase),np.cos(phase),np.sin(phase/2),np.cos(phase/2),np.sin(phase/4),np.cos(phase/4)]
                obs=np.concatenate([d.qvel[3:6]*.25,d.xmat[self.bid].reshape(3,3).T@np.array([0,0,-1]),
                    d.qpos[self.qadr]-self.default,d.qvel[self.vadr]*.05,self.current,self.last,f]).astype(np.float32)
                nxt=self.net.run(None,{self.net.get_inputs()[0].name:np.clip(obs,-100,100)[None]})[0][0]
                if not np.isfinite(nxt).all():raise RuntimeError('Nonfinite flip action')
                self.applied=self.current.copy();self.last=self.current.copy();self.current=np.clip(nxt,-100,100)
            self.slew+=np.clip(self.applied-self.slew,-.135,.135)
            vel=d.qvel[self.vadr];tau=40*(self.default+.5*self.slew-d.qpos[self.qadr])-vel
            limit=np.where(vel*tau>0,20.2,23.4)*np.clip((30-np.abs(vel))/(30-13.5),0,1)
            d.ctrl[self.aids]=np.clip(tau,-limit,limit)
            self.steps+=1
        mujoco.mj_step(m,d)
