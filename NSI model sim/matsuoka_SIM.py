import matplotlib.pyplot as plt
import math
import numpy as np
#from functions_matsuoka import matsuoka_dynamics, matsuoka_dynamics, matsuoka_coupled_oscillator, add_NSI_variable


class matsuokaOcillator():
    def __init__(self, legNumber):
        self.surface_angle = -math.pi/16
        self.legNumber = legNumber
        # synaptic weights
        self.w12 = -1.6
        self.w21 = -1.6
        self.w11 = -2.5
        self.w22 = -2.5
        self.wpos_fd = 0.25
        self.wdep_fd = 0.25
        self.wposc_fd = 0.0

        # time parameters
        self.time = 13
        self.increment = 0.01
        self.Tau = self.increment
        self.Tau_ri = 0.3
        self.Tau_ai = 6
        self.I = 1
        self.ws0 = 1

        # inputs
        # mutiplied by 100 on range because floats can not be used as increment
        # check if I made error in this for loop
        if legNumber != None:  # I know this is bad code, but I can't think of another way
            self.n = len(
                [x for x in range(100, self.time*100, int(self.increment*100))])
            self.start_time = 8.62
            self.end_time = 10.39
            self.amplitude = math.pi/8
            self.position1 = -self.amplitude
            self.position1_tr = -self.amplitude
            self.position1_c = -self.amplitude

            self.pos_goal1_c = self.amplitude
            self.pos_goal1_tr = self.amplitude
            self.pos_goal1 = self.amplitude

            self.fb1_flex = 0
            self.fb1c_flex = 0

            self.femur_tibiae_joint = femur_tibiae_class()
            self.coxa_trochanter_joint = coxa_trochanter_class(
                self.femur_tibiae_joint.a1, self.femur_tibiae_joint.a2)
            self.thorac_coxa_joint = thorac_coxa_class(
                self.femur_tibiae_joint.a1, self.femur_tibiae_joint.a2)

            self.contact = 0

    def matsuoka_dynamics(self, u1, u2, y1, y2, f1, s0, ws0, w21, w11, Tau_ri, Tau_ai, fb):
        dudt = 1/Tau_ri*(-u1+w21*y2+ws0*s0+w11*f1+fb)
        dfdt = 1/Tau_ai*(-f1+y1)
        return dudt, dfdt

    def matsuoka_coupled_oscillator(self, states, s0, fb1, fb2):
        u1 = states[0]
        u2 = states[1]
        y1 = states[2]
        y2 = states[3]
        f1 = states[4]
        f2 = states[5]

        du1dt, df1dt = self.matsuoka_dynamics(
            u1, u2, y1, y2, f1, s0, self.ws0, self.w21, self.w11, self.Tau_ri, self.Tau_ai, fb1)
        du2dt, df2dt = self.matsuoka_dynamics(
            u2, u1, y2, y1, f2, s0, self.ws0, self.w12, self.w22, self.Tau_ri, self.Tau_ai, fb2)
        return [du1dt, df1dt, du2dt, df2dt]

    # states is the list [a1,a2,y1_all,y2_all,f1,f2]
    # state is the list [a1[k-1], a2[k-1], y1,
        # y2, f1[k-1], f2[k-1]]

    def add_NSI_variable(self, states, state, d_list):
        a1 = states[0]
        a2 = states[1]
        y1_all = states[2]
        y2_all = states[3]
        f1 = states[4]
        f2 = states[5]

        a1k = state[0]
        a2k = state[1]
        f1k = state[4]
        f2k = state[5]

        da1dt = d_list[0]
        da2dt = d_list[2]
        df1dt = d_list[1]
        df2dt = d_list[3]

        k = len(a1)
        a1o = a1+[a1k+self.Tau*da1dt]
        a2o = a2+[a2k+self.Tau*da2dt]
        f1o = f1+[f1k+self.Tau*df1dt]
        f2o = f2+[f2k+self.Tau*df2dt]
        y1 = max(0, a1k)
        y2 = max(0, a2k)
        y1_allo = y1_all+[y1]
        y2_allo = y2_all+[y2]
        y1o = max(0, a1o[k])
        y2o = max(0, a2o[k])
        # returns the new list states, and the new y1,y2
        return [a1o, a2o, y1_allo, y2_allo, f1o, f2o], y1o, y2o

    def simulate(self):

        for i in range(100, self.time*100, int(self.increment*100)):
            i /= 100
            k = len(self.femur_tibiae_joint.states[0])
            print("k", k)
            feedbackInfo_list = [self.fb1_flex, self.fb1c_flex, self.wdep_fd]

            self.thorac_coxa_joint.calculateMotion1(
                k, self.I, feedbackInfo_list, self.contact, self.coxa_trochanter_joint.y1tr, self.coxa_trochanter_joint.y2tr)
            self.coxa_trochanter_joint.calculateMotion2(
                k, self.I, feedbackInfo_list)
            self.femur_tibiae_joint.calculateMotion3(
                k, self.I, feedbackInfo_list)

            self.position1_tr = self.position1_tr + self.coxa_trochanter_joint.y1tr*self.Tau
            self.position1_tr = self.position1_tr - self.coxa_trochanter_joint.y2tr*self.Tau

            if self.position1_tr >= self.surface_angle:
                self.contact = 1

            if i >= self.start_time and i <= self.end_time and self.femur_tibiae_joint.y1 > 0:
                if self.femur_tibiae_joint.y1 > 0:
                    self.position1 = self.position1 + self.femur_tibiae_joint.y1*self.Tau
                    self.position1 = self.position1 - self.femur_tibiae_joint.y2*self.Tau
                    (self.femur_tibiae_joint.position1_all).append(self.position1)
                    pos1_err = self.pos_goal1 - self.position1
                    self.femur_tibiae_joint.pos1_err_all.append(pos1_err)
                    self.fb1_flex = min(3, .1/abs(pos1_err))*self.wpos_fd
                    self.femur_tibiae_joint.fb1_flex_all.append(self.fb1_flex)

                    self.position1_c = self.position1_c + self.thorac_coxa_joint.y1c*self.Tau
                    self.position1_c = self.position1_c - self.thorac_coxa_joint.y2c*self.Tau
                    pos1c_err = self.position1_c - self.pos_goal1_c
                    self.fb1c_flex = max(-3, min(3, .1 /
                                         (pos1c_err)))*self.wposc_fd
                    self.thorac_coxa_joint.pos1c_err_all.append(pos1c_err)
                    self.thorac_coxa_joint.fb1c_flex_all.append(self.fb1c_flex)
            else:
                self.fb1_flex = 0

            k += 1


class femur_tibiae_class(matsuokaOcillator):
    def __init__(self):
        # None so that it does not get it's own copy of things such as position1, which will break things
        super().__init__(None)
        # I belive I can remove a1,a2..... and put them all in states declaration
        self.a1 = [0.01]
        self.a2 = [-0.01]
        self.f1 = [-0.01]
        self.f2 = [0.01]
        self.a1_all = []
        self.f1_all = []
        self.a2_all = []
        self.f2_all = []
        self.y1_all = []
        self.y2_all = []
        self.states = [self.a1, self.a2, self.y1_all,
                       self.y2_all, self.f1, self.f2]  # decalres states
        self.pos1_err_all = []
        self.position1_all = []
        self.fb1_flex_all = []
        self.y1 = max(0, self.a1[0])
        self.y2 = max(0, self.a2[0])
        self.count_fd_len = 0

    def calculateMotion3(self, k, I, feedbacks):
        fb1_flex = feedbacks[0]
        # state = [self.a1[k-1], self.a2[k-1], self.y1,
        # self.y2, self.f1[k-1], self.f2[k-1]]
        state = [self.states[0][k-1], self.states[1][k-1], self.y1,
                 self.y2, self.states[4][k-1], self.states[5][k-1]]
        # outputs contains [du1dt, df1dt, du2dt, df2dt]
        outputs = super().matsuoka_coupled_oscillator(state, I, -fb1_flex, fb1_flex)
        self.states, self.y1, self.y2 = super().add_NSI_variable(
            self.states, state, outputs)


class thorac_coxa_class(matsuokaOcillator):
    def __init__(self, a1, a2):  # needs a1 and a2 from femur
        super().__init__(None)
        self.a1c = [0.01]
        self.a2c = [-0.01]
        self.f1c = [-0.01]
        self.f2c = [0.01]
        self.a1c_all = []
        self.f1c_all = []
        self.a2c_all = []
        self.f2c_all = []
        self.y1c_all = []
        self.y2c_all = []
        self.pos1c_err_all = []
        self.position1c_all = []
        self.fb1c_flex_all = []
        self.states = [self.a1c, self.a2c, self.y1c_all,
                       self.y2c_all, self.f1c, self.f2c]
        self.y1c = max(0, a1[0])
        self.y2c = max(0, a2[0])
        self.countc_fd_len = 0

    def calculateMotion1(self, k, I, feedbacks, contact, y1tr, y2tr):
        fb1_flex = -feedbacks[0]
        fb1c_flex = feedbacks[1]
        wdep_fd = feedbacks[2]
        state = [self.states[0][k-1], self.states[1][k-1], self.y1c,
                 self.y2c, self.states[4][k-1], self.states[5][k-1]]
        output = super().matsuoka_coupled_oscillator(state, contact*I, -fb1_flex+y1tr *
                                                     wdep_fd-y2tr*wdep_fd-fb1c_flex, fb1_flex+fb1c_flex+y2tr*wdep_fd-y1tr*wdep_fd)
        print(y1tr, fb1_flex, fb1c_flex, y2tr, wdep_fd)
        print(y1tr, y2tr, k)

        self.states, self.y1c, self.y2c = super().add_NSI_variable(
            self.states, state, output)


class coxa_trochanter_class(matsuokaOcillator):
    def __init__(self, a1, a2):
        super().__init__(None)
        self.a1tr = [0.01]
        self.a2tr = [-0.01]
        self.f1tr = [-0.01]
        self.f2tr = [0.01]
        self.a1tr_all = []
        self.f1tr_all = []
        self.a2tr_all = []
        self.f2tr_all = []
        self.y1tr_all = []
        self.y2tr_all = []
        self.pos1tr_err_all = []
        self.position1tr_all = []
        self.fb1tr_flex_all = []
        self.states = [self.a1tr, self.a2tr, self.y1tr_all,
                       self.y2tr_all, self.f1tr, self.f2tr]
        self.y1tr = max(0, a1[0])
        self.y2tr = max(0, a2[0])
        self.counttr_fd_len = 0

    def calculateMotion2(self, k, I, feedbacks):
        fb1_flex = -feedbacks[0]
        fb1c_flex = feedbacks[1]
        # state = [self.a1tr[k-1], self.a2tr[k-1], self.y1tr,
        # self.y2tr, self.f1tr[k-1], self.f2tr[k-1]]
        state = [self.states[0][k-1], self.states[1][k-1], self.y1tr,
                 self.y2tr, self.states[4][k-1], self.states[5][k-1]]
        outputs = super().matsuoka_coupled_oscillator(state, I, -fb1_flex, fb1_flex)
        self.states, self.y1tr, self.y2tr = super().add_NSI_variable(
            self.states, state, outputs)


# graphing to test joints
leg1 = matsuokaOcillator(1)
leg1.simulate()
print("done")


xList_plot3 = []
i = 1
while i < leg1.time:
    xList_plot3.append(i)
    i += leg1.increment
xList_plot3.pop()  # WARNING: I LOST THE LAST ELEMENT
# first line we are plotting in subplot 2
yList2_1_plot3 = leg1.coxa_trochanter_joint.states[2]
# second line we are plotting in subplot 2
yList2_2_plot3 = leg1.coxa_trochanter_joint.states[3]
# first line we are plotting in subplot 3
yList3_1_plot3 = leg1.femur_tibiae_joint.states[2]
# second line we are plotting in subplot 3
yList3_2_plot3 = leg1.femur_tibiae_joint.states[3]


yList1_1_plot3 = leg1.thorac_coxa_joint.states[2]
yList1_2_plot3 = leg1.thorac_coxa_joint.states[3]


plt.subplot(3, 1, 1)
plt.plot(np.array(xList_plot3), np.array(yList3_1_plot3))
plt.plot(np.array(xList_plot3), np.array(yList3_2_plot3))


plt.xlabel("timestep")
plt.ylabel("joint position")
plt.title("femur-tibia joint position error")


plt.subplot(3, 1, 2)
plt.plot(np.array(xList_plot3), np.array(yList2_1_plot3))
plt.plot(np.array(xList_plot3), np.array(yList2_2_plot3))

plt.xlabel("timesteps")
plt.ylabel("velocity(rads/s)")
# plt.axvline(x=1)
# plt.axvline(x=7.78)
plt.legend(["y1-depressor", "y2-levator"])

plt.subplot(3, 1, 3)
plt.plot(np.array(xList_plot3), np.array(yList1_1_plot3))
plt.plot(np.array(xList_plot3), np.array(yList1_2_plot3))
plt.xlabel("timesteps")
plt.ylabel("velocity(rads/s)")
# plt.axvline(x=1)
# plt.axvline(x=7.78)
plt.legend(["y1-retractor", "y2-retractor"])

plt.show()
