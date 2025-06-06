function M = quintic(q0, qf, t0, tf)
    n = int32((tf-t0)*100);
    m = length(q0);
    t = linspace(t0,tf,n);
    
    q = zeros(n,m);
    dq = zeros(n,m);
    ddq = zeros(n,m);
    
    for i = 1:m
        a1 = (6*(q0(i) - qf(i)))/(t0 - tf)^5;
        a2 = -(15*(t0 + tf)*(q0(i) - qf(i)))/(t0 - tf)^5;
        a3 = (10*(q0(i) - qf(i))*(t0^2 + 4*t0*tf + tf^2))/(t0 - tf)^5;
        a4 =  -(30*t0*tf*(t0 + tf)*(q0(i) - qf(i)))/(t0 - tf)^5;
        a5 = (30*t0^2*tf^2*(q0(i) - qf(i)))/(t0 - tf)^5;
        a6 = -(- qf(i)*t0^5 + 5*qf(i)*t0^4*tf - 10*qf(i)*t0^3*tf^2 + 10*q0(i)*t0^2*tf^3 - 5*q0(i)*t0*tf^4 + q0(i)*tf^5)/(t0 - tf)^5;
        q(:,i) = a1*t.^5+a2*t.^4+a3*t.^3+a4*t.^2+a5*t+a6;
        dq(:,i) = 5*a1*t.^4+4*a2*t.^3+3*a3*t.^2+2*a4*t+a5;
        ddq(:,i) = 20*a1*t.^3+12*a2*t.^2+6*a3*t+2*a4;
    end
    
    t = t.';
    M = [t,q,dq,ddq];
end