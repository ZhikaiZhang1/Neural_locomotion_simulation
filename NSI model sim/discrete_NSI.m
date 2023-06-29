% excitatory + inhibitory - NOT USING
% clear all;
% close all;
% w12 = 0.78; w21 = -0.78;
% w11=1.4; w22 = 1.4;
% time = 40;
% increment = 0.01
% a1 = [0]; a2 = [0];
% o1 = [0.00001]; o2 = [-0.00001];
% 
% for i = 1:increment:time
%     k = length(o1);
%     a1_step = o1(k)+increment*(o1(k)*w11+o2(k)*w12);
%     a2_step = o2(k)+increment*(o2(k)*w22+o1(k)*w21);
%     a1 = [a1,a1_step];
%     a2 = [a2,a2_step];
%     o1 = [o1,tanh(a1(k+1))];
%     o2 = [o2,tanh(a2(k+1))];
% end
% 
% plot(1:increment:time+increment,o1)
% hold on
% plot(1:increment:time+increment,o2)
% legend("o1","o2")

% %% mutual inhibitory with self excitation
% w12 = -0.78; w21 = -0.78;
% w11=1; w22 = 1;
% time = 5;
% increment = 0.1;
% a1 = [0]; a2 = [0];
% o1 = [0.1]; o2 = [0.1];
% 
% for i = 1:increment:time
%     k = length(o1);
%     a1_step = o1(k)*w11+o2(k)*w12;
%     a2_step = o2(k)*w22+o1(k)*w21;
%     a1 = [a1,a1_step];
%     a2 = [a2,a2_step];
%     o1 = [o1,tanh(a1(k+1))];
%     o2 = [o2,tanh(a2(k+1))];
% end
% 
% plot(1:increment:time+increment,o1)
% hold on
% plot(1:increment:time+increment,o2)
% legend("o1","o2")

%% matsuoka oscillator
clear all;
close all;
multi_leg = 1;

%initial conditions / constants
surface_angle = -pi/16;


% forward kinematics and parameters
phase_offsets =[
            [pi/2,-pi/2,pi/2,-pi/2,pi/2,-pi/2];
            [0,pi,0,pi,0,pi];
            [0,pi,0,pi,0,pi]];
const_offsets = [
            [-1,0,1,-1,0,1];
            [-1,-1,-1,-1,-1,-1];
            [1,1,1,1,1,1]
        ]*pi/4;
BENDY_OUTPUT_DIS = 0.036;
BENDY_INPUT_DIS = 0.049;
ELBOW_INPUT_DIS = 0.037;
ELBOW_OUTPUT_DIS = 0.046;
FOOT_INPUT_DIS = 0.08665;
robot_height = ELBOW_OUTPUT_DIS+FOOT_INPUT_DIS;

[T02,T03,T04,T, q_1,q_2,q_3] = get_fk_sym();
%usage
% Transformation = subs(T, [q_1, q_2, q_3], [joint_angle(1),joint_angle(2),joint_angle(3)]);
% initial_pos = vpa(Transformation)*[0;0;robot_height;1];
contact_threshold = 0; % anything less than this is contact

% synaptic weights
% w12 = -2.3; w21 = -2.3;
% w11=-4; w22 = -4;
% wpos_fd = 0.4;
% wdep2ret_fd = 1;
% wlev2ret_fd = 1;
% wlev2pro_fd = 0.7;
% wposc_fd = 0.1;%0.4;
% wdep2flx_fd = 0.4;
% wlev2ext_fd = 0.4;
% wret2lev_fd = 0.05;
% wext2dep_fd = 0.25;
% P_con_w = 0;

% intra-leg weights
w12 = -2.3; w21 = -2.3;
w11=-3; w22 = -3;
wpos_fd = 0.35;
wpos_dep_constant = 0.7;
wdep2ret_fd = 1;
wlev2ret_fd = 0.6;
wlev2pro_fd = 0.3;
wpro_constant = 0.6;
wposc_fd = 0.1;%0.4;
wdep2flx_fd = 0.35;
wlev2ext_fd = 0.45;
wret2lev_fd = 0.05;
wext2dep_fd = 1.6;
wself_dep_fd = 0.3;
wext2dep_fd_constant = 0.3;
P_con_w = 0.6;

%inter-limb coordination weights
% interlimb coordination rule 1 - posterior leg lifts off inhibits anterior lifting off
w_ant_inhi_lift = 0.5;
% interlimb coordination rule 2 - posterior leg touch down induces anterior leg to protract
w_ant_exci_prot = 0.5;
% interlimb coordination rule 3 - anterior leg stance excites the posterior leg to start swing
w_post_excit_swing = 0.8;

% interlimb coordination rule 4 - TBD (not sure about the joint angle policy)
% interlimb coordination rule 4 - TBD (involves load sensing, will implement later with python sim)


% time parameters
time = 100;
increment = 0.01;
Tau = increment;
Tau_ri = 0.3;
Tau_ai = 6;
I = 1;
ws0 = 1;


% inputs
n = length(1:increment:time);
start_time = 1;
end_time = time;


amplitude = pi/8;
% front leg
position1_c = -amplitude;
position1_tr = -amplitude;
position1 = -amplitude;

pos_goal1_c = amplitude;
pos_goal1_tr = amplitude;
pos_goal1 = amplitude;

fb1_flex = 0;
fb1c_flex = 0;

% middle leg
mposition1_c = -amplitude;
mposition1_tr = -amplitude;
mposition1 = -amplitude;

mpos_goal1_c = amplitude;
mpos_goal1_tr = amplitude;
mpos_goal1 = amplitude;

mfb1_flex = 0;
mfb1c_flex = 0;


% front leg output/neuron arrays femur-tibiae
a1 = [0.01]; a2 = [-0.01];
f1 = [-0.01]; f2 = [0.08];
a1_all = [];
f1_all = [];
a2_all = [];
f2_all = [];
y1_all = [];
y2_all = [];
pos1_err_all = [];
position1_all = [];
fb1_flex_all = [];
y1 = max(0,a1(1));
y2 = max(0,a2(1));
count_fd_len = 0;

% front leg output/neuron arrays thorac-coxa
a1c = [0.01]; a2c = [-0.01];
f1c = [-0.01]; f2c = [0.08];
a1c_all = [];
f1c_all = [];
a2c_all = [];
f2c_all = [];
y1c_all = [];
y2c_all = [];
pos1c_err_all = [];
position1c_all = [];
fb1c_flex_all = [];
y1c = max(0,a1(1));
y2c = max(0,a2(1));
countc_fd_len = 0;

% front leg output/neuron arrays coxa-trochanger
a1tr = [0.01]; a2tr = [-0.01];
f1tr = [-0.01]; f2tr = [0.08];
a1tr_all = [];
f1tr_all = [];
a2tr_all = [];
f2tr_all = [];
y1tr_all = [];
y2tr_all = [];
pos1tr_err_all = [];
position1tr_all = [];
fb1tr_flex_all = [];
y1tr = max(0,a1(1));
y2tr = max(0,a2(1));
stance_phase_all = [];
counttr_fd_len = 0;

contact = 0;
contact_all = [];
contact_all_time = [];
lift_all_time = [];
z_all = [];
p_con_fd = 0;

% middle leg output/neuron arrays femur-tibiae
ma1 = [0.01]; ma2 = [-0.01];
mf1 = [-0.01]; mf2 = [0.08];
ma1_all = [];
mf1_all = [];
ma2_all = [];
mf2_all = [];
my1_all = [];
my2_all = [];
mpos1_err_all = [];
mposition1_all = [];
mfb1_flex_all = [];
my1 = max(0,a1(1));
my2 = max(0,a2(1));
mcount_fd_len = 0;

% middle leg output/neuron arrays thorac-coxa
ma1c = [0.01]; ma2c = [-0.01];
mf1c = [-0.01]; mf2c = [0.08];
ma1c_all = [];
mf1c_all = [];
ma2c_all = [];
mf2c_all = [];
my1c_all = [];
my2c_all = [];
mpos1c_err_all = [];
mposition1c_all = [];
mfb1c_flex_all = [];
my1c = max(0,a1(1));
my2c = max(0,a2(1));
mcountc_fd_len = 0;

% middle leg output/neuron arrays coxa-trochanger
ma1tr = [0.01]; ma2tr = [-0.01];
mf1tr = [-0.01]; mf2tr = [0.08];
ma1tr_all = [];
mf1tr_all = [];
ma2tr_all = [];
mf2tr_all = [];
my1tr_all = [];
my2tr_all = [];
mpos1tr_err_all = [];
mposition1tr_all = [];
mfb1tr_flex_all = [];
my1tr = max(0,a1(1));
my2tr = max(0,a2(1));
mstance_phase_all = [];
mcounttr_fd_len = 0;

mcontact = 0;
mcontact_all = [];
mcontact_all_time = [];
mlift_all_time = [];
mz_all = [];
mp_con_fd = 0;

for i = 1:increment:time
   k = length(a1);
   
   % front legs
       % thorac-coxa joint
    
       stance_ret_fd = y1tr*wdep2ret_fd*contact-y2tr*wlev2pro_fd -w_ant_exci_prot*(mcontact)*multi_leg; %+ w_ant_inhi_lift*(1-mcontact)%maybe here should have load info???
       stance_pro_fd = -y1tr*wdep2ret_fd*contact+y2tr*wlev2pro_fd*wpro_constant+w_ant_exci_prot*(mcontact)*multi_leg;%-w_ant_inhi_lift*(1-mcontact)
       swing_ret_fd = -y2tr*wlev2ret_fd*(1-contact);
       swing_pro_fd = y2tr*wlev2ret_fd*(1-contact);
    
       ret_fd = stance_ret_fd+swing_ret_fd;
       pro_fd = stance_pro_fd+swing_pro_fd;
       [da1cdt,df1cdt,da2cdt,df2cdt] = matsuoka_coupled_oscillator(a1c(k),a2c(k),y1c,y2c,f1c(k),f2c(k),I,ws0,w12,w21,w11,w22,Tau_ri,Tau_ai,ret_fd,pro_fd);
       [a1c,a2c,f1c,f2c,y1c_all,y2c_all,y1c,y2c] = add_NSI_variable(a1c,a2c,f1c,f2c,y1c_all,y2c_all,a1c(k),a2c(k),f1c(k),f2c(k),Tau,da1cdt,da2cdt,df1cdt,df2cdt);
    
        % coxa-trochanter joint
    
       stance_dep_fd = -fb1_flex-y1c*wret2lev_fd*contact+wself_dep_fd*y1tr*contact + w_ant_inhi_lift*(1-mcontact)*multi_leg;
       stance_lev_fd = fb1_flex+y1c*wret2lev_fd*contact/wext2dep_fd_constant-wself_dep_fd*y1tr*contact*wext2dep_fd_constant - w_ant_inhi_lift*(1-mcontact)*multi_leg;
       swing_dep_fd = wext2dep_fd*y2*(1-contact);
       swing_lev_fd = -wext2dep_fd*y2*(1-contact)*wext2dep_fd_constant;
    
        dep_fd = stance_dep_fd+swing_dep_fd;
        lev_fd = stance_lev_fd+swing_lev_fd;
       [da1trdt,df1trdt,da2trdt,df2trdt] = matsuoka_coupled_oscillator(a1tr(k),a2tr(k),y1tr,y2tr,f1tr(k),f2tr(k),I,ws0,w12,w21,w11,w22,Tau_ri,Tau_ai,dep_fd,lev_fd);
       [a1tr,a2tr,f1tr,f2tr,y1tr_all,y2tr_all,y1tr,y2tr] = add_NSI_variable(a1tr,a2tr,f1tr,f2tr,y1tr_all,y2tr_all,a1tr(k),a2tr(k),f1tr(k),f2tr(k),Tau,da1trdt,da2trdt,df1trdt,df2trdt);
    
       
       % femur-tibia joint
       stance_flx_fd = p_con_fd-fb1_flex+wdep2flx_fd*y1tr*contact;
       stance_ext_fd = -p_con_fd+fb1_flex-wdep2flx_fd*y1tr*contact;
       swing_flx_fd = -wlev2ext_fd*y2tr*(1-contact);
       swing_ext_fd = +wlev2ext_fd*y2tr*(1-contact);
       flx_fd = stance_flx_fd+swing_flx_fd;
       ext_fd = stance_ext_fd+swing_ext_fd;
       [da1dt,df1dt,da2dt,df2dt] = matsuoka_coupled_oscillator(a1(k),a2(k),y1,y2,f1(k),f2(k),I,ws0,w12,w21,w11,w22,Tau_ri,Tau_ai,flx_fd,ext_fd);
       [a1,a2,f1,f2,y1_all,y2_all,y1,y2] = add_NSI_variable(a1,a2,f1,f2,y1_all,y2_all,a1(k),a2(k),f1(k),f2(k),Tau,da1dt,da2dt,df1dt,df2dt); 
    
       % integrating position and create position feedbck for PEP for stance phase
       [position1,position1_all] = get_vel_pos(position1, Tau, y1,y2,position1_all);
       flex_pos_err = pos_goal1-position1;
       [fb1_flex, pos1_err_all,fb1_flex_all,ft_influence] = pos_err_fd(pos_goal1, position1, pos1_err_all, 3, 0.15, wpos_fd, 'min', fb1_flex_all);
       p_con_fd = P_control(pos_goal1,position1,P_con_w);
    
       [position1_c,position1c_all] = get_vel_pos(position1_c, Tau, y1c,y2c,position1c_all);
       stance_phase = detect_phase(y1,y2) == 0;% && detect_phase(y1tr,y2tr) == 0;
       stance_phase_all = [stance_phase_all,stance_phase];
    
       [position1_tr,position1tr_all] = get_vel_pos(position1_tr, Tau, y1tr,y2tr,position1tr_all);
    
       % detect contact
       [contact,contact_all,z_all] = detect_contact_fk(T,q_1, q_2, q_3, position1_c,position1_tr,position1, 0, contact_all,z_all,robot_height);
       
       [contact_all_time,lift_all_time] = log_contact_lift_event(contact_all,contact_all_time,lift_all_time,i,contact);
    
       if stance_phase
           
           [fb1c_flex, pos1c_err_all,fb1c_flex_all,tc_influence] = pos_err_fd(pos_goal1_c, position1_c, pos1c_err_all, 3, 0.1, wposc_fd, 'max', fb1c_flex_all);
    
       else
           % if not in stance phase, we don't put this position here
           [fb1c_flex, pos1c_err_all,fb1c_flex_all,tc_influence] = pos_err_fd(pos_goal1_c, position1_c, pos1c_err_all, 3, 0.1, wposc_fd, 'max', fb1c_flex_all);
           fb1_flex = 0;
           p_con_fd = 0;
       end
   
       
   % middle legs
       % thorac-coxa joint
    if (multi_leg == 1)
       mstance_ret_fd = my1tr*wdep2ret_fd*mcontact-my2tr*wlev2pro_fd;
       mstance_pro_fd = -my1tr*wdep2ret_fd*mcontact+my2tr*wlev2pro_fd*wpro_constant;
       mswing_ret_fd = -my2tr*wlev2ret_fd*(1-mcontact);
       mswing_pro_fd = my2tr*wlev2ret_fd*(1-mcontact);
    
       mret_fd = mstance_ret_fd+mswing_ret_fd;
       mpro_fd = mstance_pro_fd+mswing_pro_fd;
       [mda1cdt,mdf1cdt,mda2cdt,mdf2cdt] = matsuoka_coupled_oscillator(ma1c(k),ma2c(k),my1c,my2c,mf1c(k),mf2c(k),I,ws0,w12,w21,w11,w22,Tau_ri,Tau_ai,mret_fd,mpro_fd);
       [ma1c,ma2c,mf1c,mf2c,my1c_all,my2c_all,my1c,my2c] = add_NSI_variable(ma1c,ma2c,mf1c,mf2c,my1c_all,my2c_all,ma1c(k),ma2c(k),mf1c(k),mf2c(k),Tau,mda1cdt,mda2cdt,mdf1cdt,mdf2cdt);
    
        % coxa-trochanter joint
    
       mstance_dep_fd = -mfb1_flex-my1c*wret2lev_fd*mcontact+wself_dep_fd*y1tr*mcontact-(max(pi/4,flex_pos_err)-flex_pos_err)*w_post_excit_swing*contact;
       mstance_lev_fd = mfb1_flex+my1c*wret2lev_fd*mcontact/wext2dep_fd_constant-wself_dep_fd*my1tr*mcontact*wext2dep_fd_constant+(max(pi/4,flex_pos_err)-flex_pos_err)*w_post_excit_swing*contact;
       mswing_dep_fd = wext2dep_fd*my2*(1-mcontact);
       mswing_lev_fd = -wext2dep_fd*my2*(1-mcontact)*wext2dep_fd_constant;
    
        mdep_fd = mstance_dep_fd+mswing_dep_fd;
        mlev_fd = mstance_lev_fd+mswing_lev_fd;
       [mda1trdt,mdf1trdt,mda2trdt,mdf2trdt] = matsuoka_coupled_oscillator(ma1tr(k),ma2tr(k),my1tr,my2tr,mf1tr(k),mf2tr(k),I,ws0,w12,w21,w11,w22,Tau_ri,Tau_ai,mdep_fd,mlev_fd);
       [ma1tr,ma2tr,mf1tr,mf2tr,my1tr_all,my2tr_all,my1tr,my2tr] = add_NSI_variable(ma1tr,ma2tr,mf1tr,mf2tr,my1tr_all,my2tr_all,ma1tr(k),ma2tr(k),mf1tr(k),mf2tr(k),Tau,mda1trdt,mda2trdt,mdf1trdt,mdf2trdt);
    
       
       % femur-tibia joint
       mstance_flx_fd = mp_con_fd-mfb1_flex+wdep2flx_fd*my1tr*mcontact;
       mstance_ext_fd = -mp_con_fd+mfb1_flex-wdep2flx_fd*my1tr*mcontact;
       mswing_flx_fd = -wlev2ext_fd*my2tr*(1-mcontact);
       mswing_ext_fd = +wlev2ext_fd*my2tr*(1-mcontact);
       mflx_fd = mstance_flx_fd+mswing_flx_fd;
       mext_fd = mstance_ext_fd+mswing_ext_fd;
       [mda1dt,mdf1dt,mda2dt,mdf2dt] = matsuoka_coupled_oscillator(ma1(k),ma2(k),my1,my2,mf1(k),mf2(k),I,ws0,w12,w21,w11,w22,Tau_ri,Tau_ai,mflx_fd,mext_fd);
       [ma1,ma2,mf1,mf2,my1_all,my2_all,my1,my2] = add_NSI_variable(ma1,ma2,mf1,mf2,my1_all,my2_all,ma1(k),ma2(k),mf1(k),mf2(k),Tau,mda1dt,mda2dt,mdf1dt,mdf2dt); 
    
       % integrating position and create position feedbck for PEP for stance phase
       [mposition1,mposition1_all] = get_vel_pos(mposition1, Tau, my1,my2,mposition1_all);
       [mfb1_flex, mpos1_err_all,mfb1_flex_all,mft_influence] = pos_err_fd(mpos_goal1, mposition1, mpos1_err_all, 3, 0.15, wpos_fd, 'min', mfb1_flex_all);
       mp_con_fd = P_control(mpos_goal1,mposition1,P_con_w);
    
       [mposition1_c,mposition1c_all] = get_vel_pos(mposition1_c, Tau, my1c,my2c,mposition1c_all);
       mstance_phase = detect_phase(my1,my2) == 0;% && detect_phase(y1tr,y2tr) == 0;
       mstance_phase_all = [mstance_phase_all,mstance_phase];
    
       [mposition1_tr,mposition1tr_all] = get_vel_pos(mposition1_tr, Tau, my1tr,my2tr,mposition1tr_all);
    
       % detect contact
       [mcontact,mcontact_all,mz_all] = detect_contact_fk(T,q_1, q_2, q_3, mposition1_c,mposition1_tr,mposition1, 0, mcontact_all,mz_all,robot_height);
       
       [mcontact_all_time,mlift_all_time] = log_contact_lift_event(mcontact_all,mcontact_all_time,mlift_all_time,i,mcontact);
    
       if mstance_phase
           
           [mfb1c_flex, mpos1c_err_all,mfb1c_flex_all,mtc_influence] = pos_err_fd(mpos_goal1_c, mposition1_c, mpos1c_err_all, 3, 0.1, wposc_fd, 'max', mfb1c_flex_all);
    
       else
           % if not in stance phase, we don't put this position here
           [mfb1c_flex, mpos1c_err_all,mfb1c_flex_all,mtc_influence] = pos_err_fd(mpos_goal1_c, mposition1_c, mpos1c_err_all, 3, 0.1, wposc_fd, 'max', mfb1c_flex_all);
           mfb1_flex = 0;
           mp_con_fd = 0;
       end
    end
end

%front leg plots
    % errors
    % femur-tibia
    figure();
    subplot(3,1,1)
    plot(start_time:increment:start_time+(length(pos1_err_all)-1)*increment,pos1_err_all)
    title("Femur-tibia joint position error");
    ylabel("Joint position error (rad)")
    xlabel("timestep")
    
    subplot(3,1,2)
    plot(start_time:increment:start_time+(length(pos1_err_all)-1)*increment,position1_all)
    yline(pi/8,'--','PEP');
    title("Femur-tibia joint position");
    ylabel("Joint position (rad)")
    xlabel("timestep")
    
    subplot(3,1,3)
    plot(start_time:increment:start_time+(length(pos1_err_all)-1)*increment,fb1_flex_all)
    title("Femur-tibia joint position based feedback");
    ylabel("feedback signal")
    xlabel("timestep")
    set(gcf,'color','w')
    
    %thorac-coxa
    figure();
    subplot(2,1,1)
    plot(start_time:increment:start_time+(length(pos1c_err_all)-1)*increment,pos1c_err_all)
    title("position error");
    % subplot(3,1,2)
    % plot(start_time:increment:start_time+(length(pos1_err_all)-1)*increment,position1_all)
    % title("position");
    subplot(2,1,2)
    plot(start_time:increment:start_time+(length(pos1c_err_all)-1)*increment,fb1c_flex_all)
    title("feedback");
    set(gcf,'color','w')
    
    % output
    figure();
    
    subplot(3,1,1)
    plot(1:increment:time,y1c_all)
    
    
    % xline(lift_all_time,'--',{'lift'});
    % xline(7.78,'--',{'with','position fb'});
    % legend("y1 - retractor",'','')
    hold on
    plot(1:increment:time,y2c_all)
    xline(contact_all_time,'--',{'contact'});
    legend("y1 - retractor","y2 - protractor")
    % xlabel("timesteps")
    ylabel("velocity (rad/s)")
    title("thorac-coxa neuron output (joint velocity)")
    
    subplot(3,1,2)
    plot(1:increment:time,y1tr_all)
    hold on
    plot(1:increment:time,y2tr_all)
    xline(contact_all_time,'--',{'contact'});
    % xline(lift_all_time,'--',{'lift'});
    % xline(7.78,'--',{'with','position fb'});
    legend("y1 - depressor","y2 - levator",'','')
    % xlabel("timesteps")
    ylabel("velocity (rad/s)")
    title("coxa-trochantor neuron output (joint velocity)")
    
    subplot(3,1,3)
    plot(1:increment:time,y1_all)
    hold on
    plot(1:increment:time,y2_all)
    % xlabel("timesteps")
    ylabel("velocity (rad/s)")
    title("femur-tibia neuron output (joint velocity)")
    
    % legend.AutoUpdate = 'off';
    xline(contact_all_time,'--',{'contact'});
    % xline(lift_all_time,'--',{'lift'});
    % xline(7.78,'--',{'with','position fb'});
    legend("y1 - flexor","y2 - extensor",'','')
    xlabel("time")
    
    % subplot(4,1,4)
    % % plot(1:increment:time,stance_phase_all)
    % plot(1:increment:time,z_all)
    
    % subplot(4,1,4)
    % plot(1:increment:time,contact_all)
    % title("contact activity (high - contact, Low - no contact)")
    % xlabel("time")
    
    
    
    set(gcf,'color','w')
    
    
    % neuron activity
    figure();
    plot(1:increment:time+increment,a1)
    hold on
    plot(1:increment:time+increment,a2)
    legend("a1","a2")
    set(gcf,'color','w')
    
    condition = (Tau_ri-Tau_ai)^2>=4*Tau_ri*Tau_ai*w11;
    if condition
        disp("condition is true");
    else
        disp("condition is false");
    end


    %% middle leg plots
    %front leg plots
    % errors
    % femur-tibia
    figure();
    subplot(3,1,1)
    plot(start_time:increment:start_time+(length(mpos1_err_all)-1)*increment,mpos1_err_all)
    title("Femur-tibia joint position error");
    ylabel("Joint position error (rad)")
    xlabel("timestep")
    
    subplot(3,1,2)
    plot(start_time:increment:start_time+(length(mpos1_err_all)-1)*increment,mposition1_all)
    yline(pi/8,'--','PEP');
    title("Femur-tibia joint position");
    ylabel("Joint position (rad)")
    xlabel("timestep")
    
    subplot(3,1,3)
    plot(start_time:increment:start_time+(length(mpos1_err_all)-1)*increment,mfb1_flex_all)
    title("Femur-tibia joint position based feedback");
    ylabel("feedback signal")
    xlabel("timestep")
    set(gcf,'color','w')
    
    %thorac-coxa
    figure();
    subplot(2,1,1)
    plot(start_time:increment:start_time+(length(mpos1c_err_all)-1)*increment,mpos1c_err_all)
    title("position error");
    % subplot(3,1,2)
    % plot(start_time:increment:start_time+(length(pos1_err_all)-1)*increment,position1_all)
    % title("position");
    subplot(2,1,2)
    plot(start_time:increment:start_time+(length(mpos1c_err_all)-1)*increment,mfb1c_flex_all)
    title("feedback");
    set(gcf,'color','w')
    
    % output
    figure();
    
    subplot(3,1,1)
    plot(1:increment:time,my1c_all)
    
    
    % xline(lift_all_time,'--',{'lift'});
    % xline(7.78,'--',{'with','position fb'});
    % legend("y1 - retractor",'','')
    hold on
    plot(1:increment:time,my2c_all)
    xline(mcontact_all_time,'--',{'contact'});
    legend("y1 - retractor","y2 - protractor")
    % xlabel("timesteps")
    ylabel("velocity (rad/s)")
    title("thorac-coxa neuron output (joint velocity)")
    
    subplot(3,1,2)
    plot(1:increment:time,my1tr_all)
    hold on
    plot(1:increment:time,my2tr_all)
    xline(mcontact_all_time,'--',{'contact'});
    % xline(lift_all_time,'--',{'lift'});
    % xline(7.78,'--',{'with','position fb'});
    legend("y1 - depressor","y2 - levator",'','')
    % xlabel("timesteps")
    ylabel("velocity (rad/s)")
    title("coxa-trochantor neuron output (joint velocity)")
    
    subplot(3,1,3)
    plot(1:increment:time,my1_all)
    hold on
    plot(1:increment:time,my2_all)
    % xlabel("timesteps")
    ylabel("velocity (rad/s)")
    title("femur-tibia neuron output (joint velocity)")
    
    % legend.AutoUpdate = 'off';
    xline(mcontact_all_time,'--',{'contact'});
    % xline(lift_all_time,'--',{'lift'});
    % xline(7.78,'--',{'with','position fb'});
    legend("y1 - flexor","y2 - extensor",'','')
    xlabel("time")
    
    % subplot(4,1,4)
    % % plot(1:increment:time,stance_phase_all)
    % plot(1:increment:time,z_all)
    
    % subplot(4,1,4)
    % plot(1:increment:time,contact_all)
    % title("contact activity (high - contact, Low - no contact)")
    % xlabel("time")
    
    
    
    set(gcf,'color','w')
