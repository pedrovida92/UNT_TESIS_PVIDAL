function M = Jp(q,qp)
   %l1 = 345;
    l2 = 20;
    l3 = 260;
    l4 = 20;
    l5 = 260;
    l6 = 75+25;
    
    q1 = q(1);
    q2 = q(2);
    q3 = q(3);
    q4 = q(4);
    q5 = q(5);
    %q6 = q(6);
    
    qp1 = qp(1);
    qp2 = qp(2);
    qp3 = qp(3);
    qp4 = qp(4);
    qp5 = qp(5);
    %qp6 = qp(6);
    
    s1 = sin(q1);
    s2 = sin(q2);
    s4 = sin(q4);
    s5 = sin(q5);
    c1 = cos(q1);
    c2 = cos(q2);
    c4 = cos(q4);
    c5 = cos(q5);
    s23 = sin(q2 + q3);
    c23 = cos(q2 + q3);
    
    s1s4 = s1*s4;
    s1s4s23 = s1s4*s23;
    c1c4 = c1*c4;
    s4c1 = s4*c1;
    s1s5c23 = s1*s5*c23;
    s1s23c4 = s1*s23*c4;
    s23c1c4 = s23*c1c4;
    c1c5c23 = c1*c5*c23;
    s5c4c23 = s5*c4*c23;
    s5s23 = s5*s23;
    s23c5 = s23*c5;
    s4s5c1c23 = s4*s5*c1*c23;
    c4c5c23 = c4*c5*c23;
    c5c23 = c5*c23;
    s1c4 = s1*c4;
    s4s23c1 = s4*s23*c1;
    s5c23 = s5*c23;
    s5c1c23 = s5*c1*c23;
    s1c5c23 = s1*c5c23;
    s23c4c5 = s23*c4*c5;
    s5s23c4 = s5s23*c4;
    s1s4s5c23 = s1s4*s5c23;
    s4s5s23 = s4*s5s23;
    s4c5c23 = s4*c5c23;
    s23c1 = s23*c1;
    s4c1c23 = s4c1*c23;
    s1s23 = s1*s23;
    s1s4c23 = s1s4*c23;
    s4s23 = s4*s23;
    
    qp1s1 = qp1*s1;
    qp1c1 = qp1*c1;
    
    A1 = (s1s4s23 - c1c4);
    A2 = (s1s23c4 + s4c1);
    A3 = (s1s4 - s23c1c4);
    A4 = (l2 - l3*s2 - l4*s23 + l5*c23);
    A5 = (l3*c2 + l4*c23 + l5*s23 + l6*(s5c4c23 + s23c5));
    A6 = (l4*c23 + l5*s23 + l6*(s5c4c23 + s23c5));
    A7 = l6*qp4*s4s5c1c23 + l6*qp5*(s5s23 - c4c5c23)*c1;
    A8 = qp1*(l3*c2 + l4*c23 + l5*s23 + l6*s5c4c23 + l6*s23c5);
    A9 = qp2*(l3*s2 + l4*s23 - l5*c23 + l6*s5s23c4  - l6*c5c23);
    A10 = (l4*s23 - l5*c23 + l6*s5s23c4  - l6*c5c23);
    A11 = (s1c4 + s4s23c1);
    A12 = (s5s23 - c4c5c23);
    A13 = l6*qp4*s4s5s23 - l6*qp5*(s5c23 + s23c4c5);
    A14 = (s5c4c23 + s23c5);
    A15 = (s5s23c4  - c5c23);
    
    B1 = qp3*A6;
    B2 = A1*s5;
    B3 = A1*c5;
    B4 = A2*c5;
    B5 = A2*s5;
    B6 = A3*s5;
    B7 = A3*c5;
    B8 = A10*c1;
    B9 = A10*s1;
    B10 = A11*s5;
    B11 = A11*c5;
    B12 = A12*c1;
    B13 = A12*s1;
    B14 = A14*c1;
    B15 = A14*s1;
    
    C1 = (B4 + s1s5c23);
    C2 = (B5 - s1c5c23);
    C3 = (B6 + c1c5c23);
    C4 = (B7 - s5c1c23);
    C5 = qp3*B8;
    C6 = qp3*B9;
    C7 = qp4*B10;
    C8 = l6*qp5*B13;
    
    M =[-l6*qp4*B2 + l6*qp5*C1 - qp1*(l6*C3 + A4*c1) + qp2*A5*s1 + B1*s1,                    A7 + A8*s1 + A9*c1 + C5,                    A7 + qp1*(l4*c23 + l5*s23 + l6*s5c4c23 + l6*s23c5)*s1 + qp2*B8 + C5, l6*(-qp1*B2 + qp2*s4s5c1c23 + qp3*s4s5c1c23 - qp4*B6 + qp5*B11),                                              l6*(qp1*C1 + qp2*B12 + qp3*B12 + qp4*B11 - qp5*C3),                                                                                                        0;
              l6*C7 + l6*qp5*C4 + qp1*(l6*C2 - A4*s1) - qp2*A5*c1 - B1*c1, l6*qp4*s1s4s5c23 + C8 - A8*c1 + A9*s1 + C6, l6*qp4*s1s4s5c23 + C8 - qp1*(l4*c23 + l5*s23 + l6*s5c4c23 + l6*s23c5)*c1 + qp2*B9 + C6,  l6*(qp1*B10 + qp2*s1s4s5c23 + qp3*s1s4s5c23 + qp4*B5 + qp5*B3),                                               l6*(qp1*C4 + qp2*B13 + qp3*B13 + qp4*B3 + qp5*C2),                                                                                                        0;
                                                                        0,                          A13 - qp2*A5 - B1,                                                                      A13 - qp2*A6 - B1,      l6*(qp2*s4s5s23 + qp3*s4s5s23 - qp4*s5c4c23 - qp5*s4c5c23), -l6*(qp2*s5c23 + qp2*s23c4c5 + qp3*s5c23 + qp3*s23c4c5 + qp4*s4c5c23 + qp5*s5c4c23 + qp5*s23c5),                                                                                                        0;
                                                                        0,                                      qp1c1,                                                                                  qp1c1,                              -qp1s1*c23 - qp2*s23c1 - qp3*s23c1,                                                    -qp1*A1 + qp2*s4c1c23 + qp3*s4c1c23 - qp4*A3,  qp1*(s1*s5s23c4  - s1c5c23 + s4*s5*c1) - qp2*B14 - qp3*B14 + C7 - qp5*(-s1s4*c5 + s5c1c23 + s23c1c4*c5);
                                                                        0,                                      qp1s1,                                                                                  qp1s1,                               qp1c1*c23 - qp2*s1s23 - qp3*s1s23,                                                    qp1*A11 + qp2*s1s4c23 + qp3*s1s4c23 + qp4*A2, qp1*(s1s4*s5 - s5*s23c1c4 + c1c5c23) - qp2*B15 - qp3*B15 + qp4*B2 - qp5*(s1s5c23 + s1s23c4*c5 + s4c1*c5);
                                                                        0,                                          0,                                                                                      0,                                                 (qp2 + qp3)*c23,                                                              qp2*s4s23 + qp3*s4s23 - qp4*c4*c23,                                                             -qp2*A15 - qp3*A15 - qp4*s4*s5c23 - qp5*A12];
    
end