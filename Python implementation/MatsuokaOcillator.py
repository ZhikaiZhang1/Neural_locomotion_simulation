from matplotlib import pyplot as plt
import math
import numpy as np


class MatsuokaOcillator():
    def __init__(self, amplitude=np.pi/8, w12=-2.3,w21=-2.3,w11=-3,w22=-3,Tau_ri=0.3,Tau_ai=6,ws0=1,u1_init=0.01,u2_init=-0.01,f1_init=-0.01,f2_init=0.08):
        self.amplitude = amplitude

        # synaptic weights
        self.w12 = w12
        self.w21 = w21
        self.w11 = w11
        self.w22 = w22

        # time parameters
        self.Tau_ri = Tau_ri
        self.Tau_ai = Tau_ai
        self.I = 0
        self.ws0 = ws0

        # states
        self.u1 = u1_init
        self.u2 = u2_init
        self.f1 = f1_init
        self.f2 = f2_init
        self.y1 = max(self.u1,0)
        self.y2 = max(self.u2,0)
        self.du1dt = 0
        self.df1dt = 0
        self.du2dt = 0
        self.df2dt = 0

    def matsuoka_dynamics(self, s0, fb1,fb2):
        u1 = self.u1
        u2 = self.u2
        f1 = self.f1
        f2 = self.f2
        y1 = self.y1
        y2 = self.y2

        self.du1dt = 1/self.Tau_ri*(-u1+self.w21*y2+self.ws0*s0+self.w11*f1+fb1)
        self.df1dt = 1/self.Tau_ai*(-f1+y1)

        self.du2dt = 1 / self.Tau_ri * (-u2 + self.w12 * y1 + self.ws0 * s0 + self.w22 * f2 + fb2)
        self.df2dt = 1 / self.Tau_ai * (-f2 + y2)
        return
    def integrate(self,Tau):
        self.u1 = self.u1+Tau*self.du1dt
        self.u2 = self.u2+Tau*self.du2dt
        self.f1 = self.f1 + Tau * self.df1dt
        self.f2 = self.f2 + Tau * self.df2dt
        self.y1 = max(self.u1, 0)
        self.y2 = max(self.u2, 0)

    def update(self, s0, fb1, fb2,Tau):
        self.matsuoka_dynamics(s0,fb1,fb2)
        self.integrate(Tau)
        return [self.u1,self.u2,self.f1,self.f2,self.y1,self.y2]
    def get_output(self):
        return [self.y1,self.y2]
    def get_state(self):
        return [self.u1,self.u2,self.f1,self.f2]
    def set_constant(self,constant):
        # synaptic weights
        self.w12 = constant[0]
        self.w21 = constant[1]
        self.w11 = constant[2]
        self.w22 = constant[3]
        # time parameters
        self.Tau_ri = constant[4]
        self.Tau_ai = constant[5]
        self.ws0 = constant[6]
        # other
        self.amplitude = constant[7]
