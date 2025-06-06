function q = ikine(T, h, c, m)
    l1 = 345;
    l2 = 20;
    l3 = 260;
    l4 = 20;
    l5 = 260;
    l6 = 75+25;
    
    h = -1*(-1)^h;
    c = -1*(-1)^c;
    m = -1*(-1)^m;
    
    P = T(1:3,4);
    Z6 = T(1:3,3);
    Pm = P - l6*Z6;     % SUM: 0, RES: 3, MUL: 3, DIV: 0, SQRT: 0; TRIG: 0
    
    pmx = Pm(1);
    pmy = Pm(2);
    pmz = Pm(3);
    
    q1 = evalz(atan2(h*pmy,h*pmx));     % SUM: 0, RES: 0, MUL: 2, DIV: 0, SQRT: 0; TRIG: 1
    A = h*sqrt(pmx^2 + pmy^2);          % SUM: 1, RES: 0, MUL: 3, DIV: 0, SQRT: 1; TRIG: 0
    B = (A-l2)^2 + (pmz-l1)^2;          % SUM: 1, RES: 2, MUL: 2, DIV: 0, SQRT: 0; TRIG: 0
    C = sqrt(l4^2 + l5^2);              % SUM: 1, RES: 0, MUL: 2, DIV: 0, SQRT: 1; TRIG: 0
    alpha = atan2(l5,l4);               % SUM: 0, RES: 0, MUL: 0, DIV: 0, SQRT: 0; TRIG: 1
    cx = (B - C^2 - l3^2)/(2*l3*C);     % SUM: 0, RES: 2, MUL: 4, DIV: 1, SQRT: 0; TRIG: 0
    sx = c*sqrt(1-cx^2);                % SUM: 0, RES: 1, MUL: 2, DIV: 0, SQRT: 1; TRIG: 0
    q3 = evalz(alpha - atan2(sx,cx));   % SUM: 0, RES: 1, MUL: 0, DIV: 0, SQRT: 0; TRIG: 1
    c3 = cos(q3);                       % SUM: 0, RES: 0, MUL: 0, DIV: 0, SQRT: 0; TRIG: 1
    s3 = sin(q3);                       % SUM: 0, RES: 0, MUL: 0, DIV: 0, SQRT: 0; TRIG: 1
    beta = atan2(l5*c3 - l4*s3, l5*s3 + l4*c3 + l3);    % SUM: 2, RES: 1, MUL: 4, DIV: 0, SQRT: 0; TRIG: 1
    q2 = evalz(beta - atan2(A - l2, pmz - l1));         % SUM: 0, RES: 3, MUL: 0, DIV: 0, SQRT: 0; TRIG: 1
    
    T11 = T(1,1);
    T12 = T(1,2);
    T13 = T(1,3);
    
    T21 = T(2,1);
    T22 = T(2,2);
    T23 = T(2,3);
    
    T31 = T(3,1);
    T32 = T(3,2);
    T33 = T(3,3);
    
    c1 = cos(q1);       % SUM: 0, RES: 0, MUL: 0, DIV: 0, SQRT: 0; TRIG: 1
    s1 = sin(q1);       % SUM: 0, RES: 0, MUL: 0, DIV: 0, SQRT: 0; TRIG: 1
    c23 = cos(q2+q3);   % SUM: 1, RES: 0, MUL: 0, DIV: 0, SQRT: 0; TRIG: 1
    s23 = sin(q2+q3);   % SUM: 1, RES: 0, MUL: 0, DIV: 0, SQRT: 0; TRIG: 1
    
    c5 = c1*c23*T13 + s1*c23*T23 + s23*T33;         % SUM: 2, RES: 0, MUL: 5, DIV: 0, SQRT: 0; TRIG: 0
    s5 = m*sqrt(1-c5^2);                            % SUM: 0, RES: 1, MUL: 2, DIV: 0, SQRT: 1; TRIG: 0
    q5 = evalz(atan2(s5,c5));                       % SUM: 0, RES: 0, MUL: 0, DIV: 0, SQRT: 0; TRIG: 1
    c4 = m*(-c1*s23*T13 - s1*s23*T23 + c23*T33);    % SUM: 1, RES: 1, MUL: 6, DIV: 0, SQRT: 0; TRIG: 0
    s4 = m*(s1*T13 - c1*T23);                       % SUM: 0, RES: 1, MUL: 3, DIV: 0, SQRT: 0; TRIG: 0
    q4 = evalz(atan2(s4, c4));                      % SUM: 0, RES: 0, MUL: 0, DIV: 0, SQRT: 0; TRIG: 1
    c6 = m*(-c1*c23*T11 - s1*c23*T21 - s23*T31);    % SUM: 0, RES: 2, MUL: 6, DIV: 0, SQRT: 0; TRIG: 0
    s6 = m*(c1*c23*T12 + s1*c23*T22 + s23*T32);     % SUM: 2, RES: 0, MUL: 6, DIV: 0, SQRT: 0; TRIG: 0
    q6 = evalz(atan2(s6, c6));                      % SUM: 0, RES: 0, MUL: 0, DIV: 0, SQRT: 0; TRIG: 
    
    q = [q1,q2,q3,q4,q5,q6];
end