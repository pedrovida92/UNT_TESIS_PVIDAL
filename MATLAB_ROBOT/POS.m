function M = POS(q,t)
    global qc;
    global tc;
    
    t0 = tc;
    tf = t0 + t;
    q0 = qc;
    qf = q;
    
    %M = quintic(q0,qf,t0,tf);
    M = hscurve(q0,qf,50,50,t0,tf);
    qc = qf;
    tc = tf;
end