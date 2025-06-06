function M = PTP(P,O,t)
    global qc;
    global qa;
    global tc;
    T = mth(P,O);
    q0 = qc;
    t0 = tc;
    tf = t0 + t; 
    qant = q0;
    qf = iksolve(T,qant);
    %qf = ikine(T,1,1,1);
    %M = quintic(q0,qf,t0,tf);
    %M = scurve(q0,qf,50,t0,tf);
    M = hscurve(q0,qf,50,50,t0,tf);
    qa = q0;
    qc = qf;
    tc = tf;
    %robotRun(M,t0,tf)
end