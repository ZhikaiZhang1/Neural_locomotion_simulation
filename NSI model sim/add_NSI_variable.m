function [a1o,a2o,f1o,f2o,y1_allo,y2_allo,y1o,y2o] = add_NSI_variable(a1,a2,f1,f2,y1_all,y2_all,a1k,a2k,f1k,f2k,Tau,da1dt,da2dt,df1dt,df2dt)
    k = length(a1);    
    a1o = [a1,a1k+Tau*da1dt];
    a2o = [a2,a2k+Tau*da2dt];
    f1o = [f1,f1k+Tau*df1dt];
    f2o = [f2,f2k+Tau*df2dt];
    y1 = max(0,a1k);
    y2 = max(0,a2k);
    y1_allo = [y1_all,y1];
    y2_allo = [y2_all,y2];
    y1o = max(0,a1o(k+1));
    y2o = max(0,a2o(k+1));
end