from matplotlib import rcParams
import matplotlib.pyplot as plt
import math
import numpy as np
from MatsuokaOcillator import MatsuokaOcillator

class NSI_leg():
    def __init__(self, mode = 'copella_sim'):
        # simultion mode, modes = 'FK_sim' or 'copella_sim' or 'real'
        self.mode = mode
        # leg module dimensions
        self.BENDY_OUTPUT_DIS = 0.036
        self.BENDY_INPUT_DIS = 0.049
        self.ELBOW_INPUT_DIS = 0.037
        self.ELBOW_OUTPUT_DIS = 0.046
        self.FOOT_INPUT_DIS = 0.08665

        # DH parameters
        self.starting_offset = 0
        self.alpha = np.array([self.starting_offset, -np.pi / 2, 0, 0, 0])
        self.a = np.array([-self.BENDY_INPUT_DIS, -self.BENDY_OUTPUT_DIS - self.BENDY_INPUT_DIS, -self.BENDY_OUTPUT_DIS - self.BENDY_INPUT_DIS,
             -self.ELBOW_INPUT_DIS - self.BENDY_OUTPUT_DIS, -self.ELBOW_OUTPUT_DIS - self.FOOT_INPUT_DIS])
        self.d = np.array([0, 0, 0, 0, 0])


        # matsuoka oscillators
        self.thco_joint = MatsuokaOcillator()
        self.cotr_joint = MatsuokaOcillator()
        self.femtb_joint = MatsuokaOcillator()

        # interjoint weights
        self.wpos_dep_constant = 0.7
        self.wdep2ret_fd = 1
        self.wlev2ret_fd = 0.6
        self.wlev2pro_fd = 0.3
        self.wpro_constant = 0.6
        self.wpos_fd = 0.35
        self.wposc_fd = 0.1
        self.wdep2flx_fd = 0.35
        self.wlev2ext_fd = 0.45
        self.wret2lev_fd = 0.05
        self.wext2dep_fd = 1.6
        self.wself_dep_fd = 0.3
        self.wext2dep_fd_constant = 0.3
        self.P_con_w = 0.6

        # interlimb weights
        # interlimb coordination rule 1 - posterior leg lifts off inhibits anterior lifting off
        self.w_ant_inhi_lift = 0.8
        # interlimb coordination rule 2 - posterior leg touch down induces anterior leg to protract
        self.w_ant_exci_prot = 1.2
        # interlimb coordination rule 3 - anterior leg stance excites the posterior leg to start swing
        self.w_post_excit_swing = 1

        # states
        # joints pos, accel, vel, IMU, contact&load
        self.contact = 0
        self.load = 0
        self.joint_pos = np.array([0, 0, 0])
        self.joint_vel = np.array([0, 0, 0])
        self.joint_accel = np.array([0, 0, 0])
        self.eff_pos = np.array([0, 0, 0])
        return
    def detect_phase(self):
        flx_ext = self.femtb_joint.get_output()
        if flx_ext[0] <= flx_ext[1]:
            stance_wing = 1
        else:
            stance_wing = 0
        return stance_wing

    def Rotaion_Matrix(self, Q, alpha):
      R = np.array([[np.cos(Q), -np.sin(Q)*np.cos(alpha),np.sin(Q)*np.sin(alpha)],
         [np.sin(Q), np.cos(Q)*np.cos(alpha), -np.sin(alpha)*np.cos(Q)],
         [0, np.sin(alpha), np.cos(alpha)]])
      return R


    def get_FK(self, q_1, q_2, q_3, order=5):
        self.theta = np.array([self.starting_offset, q_1, q_2, q_3, np.pi / 2])
        R_1 = self.Rotaion_Matrix(self.theta[0], self.alpha[0])
        R_2 = self.Rotaion_Matrix(self.theta[1], self.alpha[1])
        R_3 = self.Rotaion_Matrix(self.theta[2], self.alpha[2])
        R_4 = self.Rotaion_Matrix(self.theta[3], self.alpha[3])
        R_5 = self.Rotaion_Matrix(self.theta[4], self.alpha[4])

        p_1 = np.array([[np.cos(self.theta[0]) * self.a[0]],[np.sin(self.theta[0]) * self.a[0]], [self.d[0]]])
        p_2 = np.array([[np.cos(self.theta[1]) * self.a[1]], [np.sin(self.theta[1]) * self.a[1]], [self.d[1]]])
        p_3 = np.array([[np.cos(self.theta[2]) * self.a[2]], [np.sin(self.theta[2]) * self.a[2]], [self.d[2]]])
        p_4 = np.array([[np.cos(self.theta[3]) * self.a[3]], [np.sin(self.theta[3]) * self.a[3]], [self.d[3]]])
        p_5 = np.array([[np.cos(self.theta[4]) * self.a[4]], [np.sin(self.theta[4]) * self.a[4]], [self.d[4]]])
        T = [np.vstack((np.concatenate((R_1,p_1),axis=1), np.array([0,0,0,1]))),
             np.vstack((np.concatenate((R_2,p_2),axis=1), np.array([0,0,0,1]))),
             np.vstack((np.concatenate((R_3,p_3),axis=1), np.array([0,0,0,1]))),
             np.vstack((np.concatenate((R_4,p_4),axis=1), np.array([0,0,0,1]))),
             np.vstack((np.concatenate((R_5,p_5),axis=1), np.array([0,0,0,1])))]
        # print("T sample: ", T[2])
        # T = T_1*T_2*T_3*T_4*T_5
        T_out = T[0]@T[1]@T[2]@T[3]@T[4]
        return T_out
        # if order == 0:
        #     return T_out
        # else:
        #     for i in range(0,order-1):
        #         T_out = T_out @ T[i+1]
        #
        #     return T_out


    def pos_err_fd(self, desired_pos, curr_pos, weight, max_min = 'min',divider=0.15,max_min_lim = 3):
        # for switching phases
        pos_err = desired_pos - curr_pos # self.femtb_joint.amplitude-self.joint_pos[2]
        if max_min == 'min':
            fd_prelim = divider / abs(pos_err)
            if pos_err>=0:
                influence = min(max_min_lim, fd_prelim)
                fd = min(max_min_lim, fd_prelim) * weight #self.wpos_fd
            else:
                influence = max_min_lim
                fd = max_min_lim * weight
        else:
            influence = max(-max_min_lim, min(max_min_lim, divider / (pos_err)))
            fd = max(-max_min_lim, min(max_min_lim, divider / (pos_err))) * weight

        return [fd, influence]

    def pos_con(self, desired_pos, pos, weight, max_min_lim=3):
        # for reaching the end pos
        output = (desired_pos - pos) * weight
        return min(output, max_min_lim)

    def test_detect_contact(self, q_1, q_2, q_3, robot_height):
        T = self.get_FK(q_1, q_2, q_3)
        initial_vec = np.array([[0],[0],[robot_height],[1]])
        self.eff_pos = T@initial_vec
        if self.eff_pos[2] > 0:
            return 0
        else:
            return 1

    def Calculate_fb(self):
        '''
        modes = 'FK_sim' 'copella_sim' or 'real'
        :return:
        '''

        # get all the outputs
        output_thco = self.thco_joint.get_output()
        output_cotr = self.cotr_joint.get_output()
        output_femtb = self.femtb_joint.get_output()


        # get flx-ext feedback
        pos_limit_fd = self.pos_err_fd(self.femtb_joint.amplitude, self.joint_pos[2], self.wpos_fd)
        p_con_fd = self.pos_con(self.femtb_joint.amplitude, self.joint_pos[2],self.P_con_w)

        if self.mode == 'FK_sim':
            self.contact = self.test_detect_contact(self.joint_pos[0],self.joint_pos[1],self.joint_pos[2],self.ELBOW_OUTPUT_DIS+self.FOOT_INPUT_DIS)
            if self.detect_phase() != 0:
                p_con_fd = 0
                pos_limit_fd[0] = 0
                pos_limit_fd[1] = 0
        elif self.mode == 'copella_sim':
            pass
        elif self.mode == 'real':
            pass

        # fb for th-co joint
        stance_ret_fd = output_cotr[0] * self.wdep2ret_fd * self.contact - output_cotr[1] * self.wlev2pro_fd
        stance_pro_fd = -output_cotr[0] * self.wdep2ret_fd * self.contact + output_cotr[1] * self.wlev2pro_fd * self.wpro_constant
        swing_ret_fd = -output_cotr[1] * self.wlev2ret_fd * (1 - self.contact)
        swing_pro_fd = output_cotr[1] * self.wlev2ret_fd * (1 - self.contact)

        ret_fd = stance_ret_fd + swing_ret_fd
        pro_fd = stance_pro_fd + swing_pro_fd


        # fb for co-tr joint
        stance_dep_fd = -pos_limit_fd[0] - output_thco[0] * self.wret2lev_fd * self.contact + self.wself_dep_fd * output_cotr[0] * self.contact
        stance_lev_fd = pos_limit_fd[0] + output_thco[0] * self.wret2lev_fd * self.contact / self.wext2dep_fd_constant - self.wself_dep_fd * output_cotr[0] * self.contact * self.wext2dep_fd_constant
        swing_dep_fd = self.wext2dep_fd * output_femtb[1] * (1 - self.contact)
        swing_lev_fd = -self.wext2dep_fd * output_femtb[1] * (1 - self.contact) * self.wext2dep_fd_constant

        dep_fd = stance_dep_fd + swing_dep_fd
        lev_fd = stance_lev_fd + swing_lev_fd

        # fb for fem-tb joint
        stance_flx_fd = p_con_fd - pos_limit_fd[0] + self.wdep2flx_fd * output_cotr[0] * self.contact
        stance_ext_fd = -p_con_fd + pos_limit_fd[0] - self.wdep2flx_fd * output_cotr[0] * self.contact
        swing_flx_fd = -self.wlev2ext_fd * output_cotr[1] * (1 - self.contact)
        swing_ext_fd = +self.wlev2ext_fd * output_cotr[1] * (1 - self.contact)
        flx_fd = stance_flx_fd + swing_flx_fd
        ext_fd = stance_ext_fd + swing_ext_fd

        return [[ret_fd,pro_fd],[dep_fd,lev_fd],[flx_fd,ext_fd]]
    def update_robot_states(self, Tau, vel_thco=None, vel_cotr=None, vel_femtb=None):
        if self.mode == 'FK_sim':
            self.joint_pos[0] += vel_thco[0]*Tau-vel_thco[1]*Tau
            self.joint_pos[1] += vel_cotr[0] * Tau - vel_cotr[1] * Tau
            self.joint_pos[2] += vel_femtb[0] * Tau - vel_femtb[1] * Tau
        elif self.mode == 'copella_sim':
            # get it from copella sim
            pass
        elif self.mode == 'real':
            # get it from robot
            pass
    def update_joints(self, Tau, feedback,I):
        thco_all_states = self.thco_joint.update(I,feedback[0][0],feedback[0][1],Tau)
        cotr_all_states = self.cotr_joint.update(I, feedback[1][0], feedback[1][1], Tau)
        femtb_all_states = self.femtb_joint.update(I, feedback[2][0], feedback[2][1], Tau)
        return np.array([thco_all_states,cotr_all_states,femtb_all_states])
    def update(self, Tau,I):
        feedback = self.Calculate_fb()
        all_curr_states = self.update_joints(Tau, feedback, I)
        self.update_robot_states(Tau, all_curr_states[0,4:6],all_curr_states[1,4:6],all_curr_states[2,4:6])
        return all_curr_states
    def get_all_robot_states(self):
        return [self.joint_pos, self.contact, self.load, self.joint_vel,self.joint_accel]
    def set_robot_state(self, pos, vel, accel, contact = 0, load = 0):
        self.joint_pos = pos
        self.joint_vel = vel
        self.joint_accel = accel
        self.contact = contact
        self.load = load




def unit_test():
    single_leg = NSI_leg(mode='FK_sim')
    single_leg.set_robot_state(np.array([-np.pi/8,-np.pi/8,-np.pi/8]),np.zeros(3),np.zeros(3))
    time = 50
    increment = 0.01
    Tau = increment
    I = 1
    N = int(time/increment)

    all_eff_pos = np.zeros(N)
    all_neuron_states = np.zeros((N, 3,2))
    all_contact = np.zeros((N, 1))
    contact_times = []
    for i in range(0,N):
        neuron_states = single_leg.update(Tau, I)
        all_neuron_states[i, 0, 0:2] = neuron_states[0, 4:6]
        all_neuron_states[i, 1, 0:2] = neuron_states[1, 4:6]
        all_neuron_states[i, 2, 0:2] = neuron_states[2, 4:6]
        all_eff_pos[i] = single_leg.eff_pos[2]
        all_contact[i] = single_leg.contact
        if i > 0 and all_contact[i-1] == 0 and all_contact[i] == 1:
            contact_times.append(i)

    contact_times = np.array(contact_times)/100
    x = np.arange(0.0, time, increment)
    plt.figure()
    plt.subplot(411)
    plt.plot(x,all_neuron_states[:,0,:])
    plt.legend(['retractor', 'protractor'])
    for index in contact_times:
        plt.axvline(x=index,color='black', linestyle='--')

    plt.subplot(412)
    plt.plot(x,all_neuron_states[:, 1, :])
    plt.legend(['depressor', 'levator'])
    for index in contact_times:
        plt.axvline(x=index,color='black', linestyle='--')

    plt.subplot(413)
    plt.plot(x,all_neuron_states[:, 2, :])
    plt.legend(['flexor', 'extensor'])
    for index in contact_times:
        plt.axvline(x=index,color='black', linestyle='--')

    plt.subplot(414)
    plt.plot(x,all_eff_pos)
    plt.axhline(y=0, color='black', linestyle='--')




    plt.plot()

    plt.show()
    return


unit_test()