function M = cubic(q0, qf, t0, tf)
    n = (tf-t0)*120;
    m = length(q0);
    t = linspace(t0,tf,n);
    
    q = zeros(n,m);
    dq = zeros(n,m);
    ddq = zeros(n,m);
    
    for i = 1:m
        a1 = -2*(q0(i) - qf(i))/(t0 - tf)^3;
        a2 = 3*(t0 + tf)*(q0(i) - qf(i))/(t0 - tf)^3;
        a3 =  -6*t0*tf*(q0(i) - qf(i))/(t0 - tf)^3;
        a4 = (qf(i)*t0^3 - 3*qf(i)*t0^2*tf + 3*q0(i)*t0*tf^2 - q0(i)*tf^3)/(t0 - tf)^3;
        q(:,i) = a1*t.^3+a2*t.^2+a3*t+a4;
        dq(:,i) = 3*a1*t.^2+2*a2*t+a3;
        ddq(:,i) = 6*a1*t + 2*a2;
    end
    
    t = t.';
    M = [t,q,dq,ddq];
end