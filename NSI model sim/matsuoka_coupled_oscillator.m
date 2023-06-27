function [du1dt,df1dt,du2dt,df2dt] = matsuoka_coupled_oscillator(u1,u2,y1,y2,f1,f2,s0,ws0,w12,w21,w11,w22, Tau_ri, Tau_ai, fb1,fb2)
    [du1dt,df1dt] = matsuoka_dynamics(u1,u2,y1,y2,f1,s0,ws0,w21,w11,Tau_ri,Tau_ai,fb1);
    [du2dt,df2dt] = matsuoka_dynamics(u2,u1,y2,y1,f2,s0,ws0,w12,w22,Tau_ri,Tau_ai,fb2);
end