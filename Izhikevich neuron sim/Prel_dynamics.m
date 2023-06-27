function dpdt = Prel_dynamics(t,tk,tauP,P0,fD,Prel,spike)
    dpdt = (Prel-P0)/tauP-fD*Prel*spike;
end