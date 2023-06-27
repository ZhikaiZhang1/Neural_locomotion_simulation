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

% synaptic weights
w12 = -2.3; w21 = -2.3;
w11=-3; w22 = -3;
wpos_fd = 0.3;

% time parameters
time = 50;
increment = 0.01;
a1 = [0.01]; a2 = [-0.01];
f1 = [0]; f2 = [0.08];
Tau = increment;
Tau_ri = 0.5;
Tau_ai = 6;
I = 1;
ws0 = 1;

% inputs
n = length(1:increment:time);
% I1=[-100*ones(1,0.1*n),-64*ones(1,0.9*n)];% pulse of input DC current
% I2=[-64*ones(1,0.1*n),-64*ones(1,0.9*n)];% pulse of input DC current

% output arrays
a1_all = [];
f1_all = [];
a2_all = [];
f2_all = [];
y1_all = [];
y2_all = [];
y1 = max(0,a1(1));
y2 = max(0,a2(1));
for i = 1:increment:time
    k = length(a1);
   [da1dt,df1dt,da2dt,df2dt] = matsuoka_coupled_oscillator(a1(k),a2(k),y1,y2,f1(k),f2(k),I,ws0,w12,w21,w11,w22,Tau_ri,Tau_ai,0,0);
   [a1,a2,f1,f2,y1_all,y2_all,y1,y2] = add_NSI_variable(a1,a2,f1,f2,y1_all,y2_all,a1(k),a2(k),f1(k),f2(k),Tau,da1dt,da2dt,df1dt,df2dt);
end

% plot(1:increment:time,max(0,y1_all-0.5))
% hold on
% plot(1:increment:time,max(0,y2_all-0.5))

% output
plot(1:increment:time,y1_all)
hold on
plot(1:increment:time,y2_all)
legend("y1 - flexor","y2 - extensor")
xlabel("timesteps")
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


%     [da1dt,df1dt] = matsuoka_dynamics(a1(k),a2(k),y1,y2,f1(k),I,ws0,w21,w11,Tau_ri,Tau_ai,0);
%     [da2dt,df2dt] = matsuoka_dynamics(a2(k),a1(k),y2,y1,f2(k),I,ws0,w12,w22,Tau_ri,Tau_ai,0);
%     a1 = [a1,a1(k)+Tau*da1dt];
%     a2 = [a2,a2(k)+Tau*da2dt];
%     f1 = [f1,f1(k)+Tau*df1dt];
%     f2 = [f2,f2(k)+Tau*df2dt];
%     y1_all = [y1_all,y1];
%     y2_all = [y2_all,y2];
%     y1 = max(0,a1(k+1));
%     y2 = max(0,a2(k+1));