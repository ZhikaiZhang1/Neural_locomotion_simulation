% % Created by Eugene M. Izhikevich, February 25, 2003
% % Excitatory neurons    Inhibitory neurons
% Ne=800;                 Ni=200;
% re=rand(Ne,1);          ri=rand(Ni,1);
% a=[0.02*ones(Ne,1);     0.02+0.08*ri];
% b=[0.2*ones(Ne,1);      0.25-0.05*ri];
% c=[-65+15*re.^2;        -65*ones(Ni,1)];
% d=[8-6*re.^2;           2*ones(Ni,1)];
% S=[0.5*rand(Ne+Ni,Ne),  -rand(Ne+Ni,Ni)];
% 
% v=-65*ones(Ne+Ni,1);    % Initial values of v
% u=b.*v;                 % Initial values of u
% firings=[];             % spike timings
% 
% for t=1:1000            % simulation of 1000 ms
%   I=[5*randn(Ne,1);2*randn(Ni,1)]; % thalamic input
%   fired=find(v>=30);    % indices of spikes
%   firings=[firings; t+0*fired,fired];
%   v(fired)=c(fired);
%   u(fired)=u(fired)+d(fired);
%   I=I+sum(S(:,fired),2);
%   v=v+0.5*(0.04*v.^2+5*v+140-u+I); % step 0.5 ms
%   v=v+0.5*(0.04*v.^2+5*v+140-u+I); % for numerical
%   u=u+a.*(b.*v-u);                 % stability
% end;
% plot(firings(:,1),firings(:,2),'.');
% plot((1:1000), v);


clear all; close all;
C=100; vr=-60; vt=-40; k=0.7; % parameters used for RS
a=1; b=1.5;c=-60; d=0; %b=-2;  % neocortical pyramidal neurons
w12=-0.78; w21=-0.78; vsyn=-vr;%vsyn=-60;  %synaptic connections, mutually inhibitory
vpeak=30; % spike cutoff
s1 = -0.5;s2=0.5;

gsyn = 1; Egaba = -65;

T=1000; tau=0.2; % time span and step (ms)
n=round(T/tau); % number of simulation steps
v1=vr*ones(1,n); u1=0*v1; v2=vr*ones(1,n); u2=0*v2; % initial values

f = 1; %Hz
I1=[-100*ones(1,0.1*n),-64*ones(1,0.9*n)];% pulse of input DC current
I2=[-64*ones(1,0.1*n),-64*ones(1,0.9*n)];% pulse of input DC current

%synaptic weights dynamics
spike_all_1 = [];
spike_all_2 = [];
P_rel_all_1 = [];
P_rel_all_2 = [];
Isyn_all_2 = [];
Isyn_all_1 = [];
Prel1 = 1;
Prel2 = 1;
fraction_P = 0.9;
tauP = 40;
recovery_period = -1;
I0_mutual = -35;
Tau_syn = 10;
I0_syn2 = 0;
I0_syn1 = 0;
t2 = 0;
t1 = 0;
Isyn2 = 0;
Isyn1 =0;
for i=1:n-1 % forward Euler method
    
    %gsyn*s2*(v1(i)-vsyn);
%     Isyn2 = 0;%gsyn*s1*(v2(i)-vsyn);
    [v1(i+1),u1(i+1),v1(i),spike1]=IZH_RS(v1(i),u1(i),vpeak,tau,a,b,c,d,Isyn1,0,I1(i));
    [v2(i+1),u2(i+1),v2(i),spike2]=IZH_RS(v2(i),u2(i),vpeak,tau,a,b,c,d,Isyn2,0,I2(i));
    spike_all_1 = [spike_all_1,spike1];
    spike_all_2 = [spike_all_2,spike2];
    
    % synaptic plasticity - depression 
    % for neuron 2
    Prel1 = Prel1+tau*Prel_dynamics(i,i,tauP,0,fraction_P,Prel1,spike1);
    if Prel1>1
        Prel1 = 1;
    end
    P_rel_all_1 = [P_rel_all_1,Prel1];
    % for neuron 1
    Prel2 = Prel2+tau*Prel_dynamics(i,i,tauP,0,fraction_P,Prel2,spike2);
    if Prel2>1
        Prel2 = 1;
    end
    P_rel_all_2 = [P_rel_all_2,Prel2];

    % synaptic current for second neuron
    if spike1 == 1
        I0_syn2 = Isyn2 + I0_mutual;
        t2 = 0;
    end
    Isyn2 = Prel1*synaptic_current_decay(I0_syn2,t2,Tau_syn,0);
    t2 = t2+tau;
    Isyn_all_2 = [Isyn_all_2,Isyn2];

    % synaptic current for first neuron
    if spike2 == 1
        I0_syn1 = Isyn1 + I0_mutual;
        t1 = 0;
    end
    Isyn1 = Prel2*synaptic_current_decay(I0_syn1,t1,Tau_syn,0);
    t1 = t1+tau;
    Isyn_all_1 = [Isyn_all_1,Isyn1];
end

% for i=n:n+recovery_period % forward Euler method
%     spike_all_1 = [spike_all_1,0];
%     Prel1 = Prel1+tau*Prel_dynamics(i,i,tauP,0,fraction_P,Prel1,0);
%     if Prel1>1
%         Prel1 = 1;
%     end
%     P_rel_all_1 = [P_rel_all_1,Prel1];
%     
% end

figure();
plot(tau*(1:n), v1); % plot the result
hold on
plot(tau*(1:n), v2); % plot the result

title('Izhikevich Half Centre Oscillator','FontSize', 14)
xlabel("timestep",'FontSize', 13)
ylabel("neuron activity (mV)",'FontSize', 13)
legend("Neuron 2","Neuron 1")
set(gcf,'color','w')


 figure();
 subplot(2,1,1);
 plot(tau*(1:n+recovery_period),P_rel_all_1);
 title('Synaptic Plasticity in Neuron 2 - Depression','FontSize', 14);
 xlabel("timestep",'FontSize', 13)
 ylabel("synaptic weight",'FontSize', 13)
 subplot(2,1,2);
 plot(tau*(1:n+recovery_period),spike_all_1);
 title('Neuron 1 Spiking Activity','FontSize', 14);
 ylabel("neuron activity (mV)",'FontSize', 13)
 xlabel("timestep",'FontSize', 13)
 set(gcf,'color','w')
% 
  figure();
 subplot(2,1,1);
 plot(tau*(1:n+recovery_period),P_rel_all_2);
 title('synaptic plasticity neuron 1 - depression');
 subplot(2,1,2);
 plot(tau*(1:n+recovery_period),spike_all_2);
 title('neuron 2 spiking activity');

 figure();
 plot(tau*(1:n-1),Isyn_all_2);
 title('synaptic current  neuron 2');

  figure();
 plot(tau*(1:n-1),Isyn_all_1);
 title('synaptic current  neuron 1');
