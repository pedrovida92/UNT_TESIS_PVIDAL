function M = CIRC(p2,p3,O,t)
    global qc;
    global qa;
    global tc;
    
    t0 = tc;
    tf = t0 + t;
    q0 = qc;
    T0 = dkine(q0);
    Tf = mth(p3,O);
    R0 = T0(1:3,1:3);
    Rf = Tf(1:3,1:3);
    P0 = T0(1:3,4);
    
    p1 = P0.';
    n = cross(p2-p1,p3-p1);
    n = n/norm(n);

    A = [p1-p2;
         p2-p3;
         n];
    B = [(norm(p1)^2-norm(p2)^2)/2;
         (norm(p2)^2-norm(p3)^2)/2;
                         dot(p1,n)];

    c = (A\B).';

    r1 = p1-c;
    r2 = p2-c;
    r3 = p3-c;
    
    ct12 = dot(r1,r2)/(norm(r1)*norm(r2));
    st12 = sqrt(1-ct12^2);
    theta12 = atan2(st12,ct12);
    ct23 = dot(r2,r3)/(norm(r2)*norm(r3));
    st23 = sqrt(1-ct23^2);
    theta23 = atan2(st23,ct23);
    theta = theta12 + theta23;
    %radio = norm(r1);
    
    %LT = quintic(0,1,t0,tf);
    %LT = scurve(0,1,50,t0,tf);
    LT = hscurve(0,1,75,75,t0,tf);
    
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
            r = (rotPar(s(i)*theta,n)*r1.').';
            P = c.' + r.';
            V = cross(n,r)*sp(i)*theta;
            A = cross(n,sp(i)*theta*V + spp(i)*theta*r);
            xep = [V.';W];
            xepp = [A.';Wp];
            T = [R0,P;zeros(1,3),1];
            q = iksolve(T,qant);
            qant = q;
            qp = J(q)\xep;
            qpp = J(q)\(xepp - Jp(q,qp)*qp);
            Q(i,:) = q;
            Qp(i,:) = qp;
            Qpp(i,:) = qpp;
            Qppp(i,:) = (qpp-qppant)*100;
            qppant = qpp;
        end
    else
        [angle,k] = Rot(R0,Rf);
        for i = 1:length(t)
            r = (rotPar(s(i)*theta,n)*r1.').';
            P = c.' + r.';
            V = cross(n,r)*sp(i)*theta;
            A = cross(n,sp(i)*theta*V + spp(i)*theta*r);
            R = R0*rotPar(s(i)*angle,k);
            W = R0*sp(i)*angle*k;
            Wp = R0*spp(i)*angle*k;
            xep = [V.';W];
            xepp = [A.';Wp];
            T = [R,P;zeros(1,3),1];
            q = iksolve(T,qant);
            qant = q;
            qp = J(q)\xep;
            qpp = J(q)\(xepp - Jp(q,qp)*qp);
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