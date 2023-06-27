function I_syn = synaptic_current_decay(I0,t,Tau_syn,base)
%     I_syn = I0*exp(1)*t*(exp(-t/Tau_syn)/Tau_syn)+base;
    I_syn = I0*(exp(-t/Tau_syn))+base;
end