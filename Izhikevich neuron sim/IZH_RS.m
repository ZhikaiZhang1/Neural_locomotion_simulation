function [v_out,u_out,v_prev, spike] = IZH_RS(v,u,vpeak,Tau,a,b,c,d,I_inh,I_ext,I_bias)
    v_out = v+Tau*(0.04*v^2+5*v+140-u+I_inh+I_ext+I_bias);
    u_out=u+Tau*a*(b*v-u);
    if v>vpeak
        v_prev=vpeak; % padding the spike amplitude
        v_out=c; % membrane voltage reset
        u_out=u_out+d; % recovery variable update
        spike = 1;
    else
        v_prev = v;
        spike = 0;
    end 


end