# SPDX-License-Identifier: BSD-3-Clause
# Policy observation semantics adapted from Sang-SC/go2_quad2hand (2026).
# See third_party/quad2hand-LICENSE.txt and docs/courtyard-showcase.md.
"""Pinned quad/handstand inference with named-joint mapping, simulation only."""
from pathlib import Path
import numpy as np
import mujoco
import onnxruntime as ort

class Quad2Hand:
    def __init__(self, model, data, assets):
        self.model,self.data=model,data
        names=[leg+'_'+joint+'_joint' for leg in ('FL','FR','RL','RR')
               for joint in ('hip','thigh','calf')]
        jids=[model.joint(n).id for n in names]
        self.qadr=model.jnt_qposadr[jids];self.vadr=model.jnt_dofadr[jids]
        self.aids=[model.actuator(n.removesuffix('_joint')).id for n in names]
        self.bid=model.body('base_link').id
        options=ort.SessionOptions();options.intra_op_num_threads=1;options.inter_op_num_threads=1
        folder=Path(assets)/'quad2hand'
        self.est=ort.InferenceSession(str(folder/'go2_estimator.onnx'),options,providers=['CPUExecutionProvider'])
        self.actor=ort.InferenceSession(str(folder/'go2_actor.onnx'),options,providers=['CPUExecutionProvider'])
        self.default=np.tile([0.,.8,-1.5],4)
        self.reset()

    def reset(self):
        self.action=np.zeros(12,dtype=np.float32)
        self.history=np.zeros((4,55),dtype=np.float32)
        self.first=True;self.steps=0

    def step(self, mode, command):
        m,d=self.model,self.data
        original_dt=m.opt.timestep
        # Two 2.5-ms substeps preserve the runtime's overall 5-ms clock.
        m.opt.timestep=original_dt/2
        try:
            for _ in range(2):
                t=self.steps*m.opt.timestep
                if self.steps%8==0:
                    phase=(t%(.4 if mode else .5))/(.4 if mode else .5)
                    leg=(phase+np.array([0,.5,0,0] if mode else [0,.5,.5,0]))%1
                    obs=np.concatenate([d.qvel[3:6]*.25,
                        d.xmat[self.bid].reshape(3,3).T@np.array([0.,0.,-1.]),
                        np.asarray(command)*[2,2,.25],d.qpos[self.qadr]-self.default,
                        d.qvel[self.vadr]*.05,self.action,
                        np.sin(2*np.pi*leg),np.cos(2*np.pi*leg),[1-mode,mode]]).astype(np.float32)
                    estin=np.concatenate([obs,self.history.ravel()]).astype(np.float32)
                    if self.first:self.history[:]=obs;self.first=False
                    else:self.history[1:]=self.history[:-1].copy();self.history[0]=obs
                    latent=self.est.run(None,{'input':estin[None]})[0].ravel()
                    actin=np.concatenate([estin,latent]).astype(np.float32)
                    self.action=self.actor.run(None,{'input':actin[None]})[0].ravel()
                    if not np.isfinite(self.action).all():raise RuntimeError('Nonfinite handstand action')
                torque=30*(self.default+.25*self.action-d.qpos[self.qadr])-.75*d.qvel[self.vadr]
                d.ctrl[self.aids]=np.clip(torque,-23.5,23.5)
                mujoco.mj_step(m,d);self.steps+=1
        finally:
            m.opt.timestep=original_dt
