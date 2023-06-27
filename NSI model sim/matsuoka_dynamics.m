function [dudt,dfdt] = matsuoka_dynamics(u1,u2,y1,y2,f1,s0,ws0,w21,w11, Tau_ri, Tau_ai, fb)
    dudt = 1/Tau_ri*(-u1+w21*y2+ws0*s0+w11*f1+fb);
    dfdt = 1/Tau_ai*(-f1+y1);
end