function M = LIN(P,O,t)
    global qc;
    global qa;
    global tc;
    
    t0 = tc;
    tf = t0 + t;
    q0 = qc;
    T0 = dkine(q0);
    Tf = mth(P,O);
    
    R0 = T0(1:3,1:3);
    Rf = Tf(1:3,1:3);
    P0 = T0(1:3,4);
    Pf = Tf(1:3,4);
    d = Pf-P0;
    L = norm(d);
    u = d/L;
    
    %LT = quintic(0,1,t0,tf);
    %LT = scurve(0,1,50,t0,tf);
    LT = hscurve(0,1,50,50,t0,tf);
    
    t = LT(:,1);
    s = LT(:,2);
    sp = LT(:,3);
    spp = LT(:,4);
    
    qant = q0;
    qppant = zeros(6,1);
    Q = zeros(length(t),6);
    Qp = zeros(length(t),6);
    Qpp = zeros(length(t),6);
    Qppp = zeros(length(t),6);
    
    W = [0,0,0].';
    Wp = [0,0,0].';
    
    if isequal(round(R0,4),round(Rf,4))
        for i = 1:length(t)
            P = P0 + s(i)*L*u;
            V = sp(i)*L*u;
            A = spp(i)*L*u;
            xep = [V;W];
            xepp = [A;Wp];
            T = [R0,P;zeros(1,3),1];
            q = iksolve(T,qant);
            qp = J(q)\xep;
            qpp = J(q)\(xepp - Jp(q,qp)*qp);
            qant = q;
            Q(i,:) = q;
            Qp(i,:) = qp;
            Qpp(i,:) = qpp;
            Qppp(i,:) = (qpp-qppant)*100;
            qppant = qpp;
        end
    else
        [theta,r] = Rot(R0,Rf);
        for i = 1:length(t)
            P = P0 + s(i)*L*u;
            V = sp(i)*L*u;
            A = spp(i)*L*u;
            R = R0*rotPar(s(i)*theta,r);
            W = R0*sp(i)*theta*r;
            Wp = R0*spp(i)*theta*r;
            xep = [V;W];
            xepp = [A;Wp];
            T = [R,P;zeros(1,3),1];
            q = iksolve(T,qant);
            qp = J(q)\xep;
            qpp = J(q)\(xepp - Jp(q,qp)*qp);
            qant = q;
            Q(i,:) = q;
            Qp(i,:) = qp;
            Qpp(i,:) = qpp;
            Qppp(i,:) = (qpp-qppant)*100;
            qppant = qpp;
        end
    end
    qa = q0;
    qc = Q(end,:).';
    tc = tf;
    M = [t,Q,Qp,Qpp,Qppp];
    %robotRun(M,t0,tf)
end