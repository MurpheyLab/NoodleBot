#!/usr/bin/env python3
'''
Pinosky, A., Abraham, I., Broad, A., Argall, B. and Murphey, T.D., 2023. 
Hybrid control for combining model-based and model-free reinforcement learning. 
The International Journal of Robotics Research, 42(6), pp.337-355.
'''
import torch
from torch.distributions import Normal
import time
import numpy as np
from utils import _batch_mv

class StochPolicyWrapper(object):

    def __init__(self, model, policy, samples=10, horizon=10, lam=0.1,
                 barrier=None,bound=1e10,device='cpu',
                 use_real_env=False,cost_to_go=False,
                 receding=False,gamma=1.0, tensor=False):

        self.device         = device
        self.tensor         = tensor

        self.model          = model
        self.policy         = policy
        self.num_actions    = model.num_actions
        self.t_H            = horizon
        self.lam            = lam
        self.samples        = samples
        self.bound          = bound

        if barrier is not None:
            raise NotImplementedError
        if use_real_env:
            raise NotImplementedError

        self.cost_to_go      = cost_to_go
        self.gamma           = gamma
        self.receding        = receding

        self.set_horizon_params()

    def reset(self):
        with torch.no_grad():
            self.a.zero_()

    def set_horizon_params(self,print_params=False,copy_a=False):
        gammas               = self.gamma**torch.arange(self.t_H,device=self.device)
        self.gammas          = gammas.unsqueeze(-1).repeat(1,self.samples)
        if copy_a:
            old_a = self.a.clone()
        self.a               = torch.zeros(self.t_H, self.num_actions,device=self.device)
        if copy_a:
            self.a[:old_a.shape[0]] = old_a

    def update_horizon(self,horizon):
        self.t_H = horizon
        self.set_horizon_params(copy_a=True)

    def __call__(self, state, eval=False):
        with torch.no_grad():
            self.a[:-1] = self.a[1:].clone()
            self.a[-1].zero_()

            s0 = torch.FloatTensor(state).unsqueeze(0).to(self.device)
            s = s0.repeat(self.samples, 1)
            mu, log_std = self.policy(s)

            sk = torch.zeros(self.t_H,self.samples,device=self.device)
            da = torch.zeros(self.t_H,self.samples,self.num_actions,device=self.device)
            log_prob = torch.zeros(self.t_H,self.samples,device=self.device)

            for t in range(self.t_H):
                pi = Normal(mu,log_std.exp())
                v = pi.sample()
                da_t = v - self.a[t].expand_as(v)
                log_prob[t] = pi.log_prob(da_t).sum(1)
                da[t] = da_t
                s, _, rew, _ = self.model(s, v)
                s = torch.clamp(s,-self.bound,self.bound) # prevent crazy unbounded stuff from happening
                mu, log_std = self.policy(s)
                sk[t] = rew.squeeze()

            if self.receding:
                sk = sk*self.gammas

            if self.cost_to_go:
                sk = torch.cumsum(sk.flip(0), 0).flip(0)
                sk = sk + self.lam*log_prob 
                sk = sk - torch.max(sk, dim=1, keepdim=True)[0]
                w = torch.exp(sk.div(self.lam)) + 1e-5 
                w.div_(torch.sum(w, dim=1, keepdim=True))
                self.a = self.a + _batch_mv(torch.transpose(da,-1,-2), w)
            else:
                sk = sk + self.lam*log_prob 
                sk = torch.sum(sk,0)
                sk = sk - torch.max(sk)
                w = torch.exp(sk.div(self.lam)) + 1e-5
                w.div_(torch.sum(w))
                self.a = self.a + torch.transpose(da,-1,-2) @ w

            if self.tensor:
                return self.a[0].detach().clone()
            else:
                return self.a[0].cpu().clone().numpy()
