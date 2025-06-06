function M = HOME(t)
    global qc;
    global qa;
    global tc;
    t0 = tc;
    tf = t0 + t;
    q0 = qc;
    qf = [0 0 0 0 0 0];
    %M = quintic(q0,qf,t0,tf);
    %M = scurve(q0,qf,50,t0,tf);
    M = hscurve(q0,qf,50,50,t0,tf);
    qa = q0;
    qc = qf;
    tc = tf;
    %robotRun(M,t0,tf)
end