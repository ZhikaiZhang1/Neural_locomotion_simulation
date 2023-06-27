function dadt = NSI_dynamics(A,B,I,w21,w11, Tau)
    I_syn = w21*B;
    dadt = 1/Tau*(I+I_syn+w11*A);
    
end