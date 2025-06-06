function M = J(q)
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
  % q6 = q(6);
    
    s1 = sin(q1);
    s2 = sin(q2);
    s4 = sin(q4);
    s5 = sin(q5);
    
    c1 = cos(q1);
    c2 = cos(q2);
    c4 = cos(q4);
    c5 = cos(q5);
    
    s23 = sin(q2+q3);
    c23 = cos(q2+q3);
        
    s5s23 = s5*s23;
    s4c1 = s4*c1;
    c1c23 = c1*c23;
    s1s23c4 = s1*s23*c4;
    s1c4 = s1*c4;
    s5c4c23 = s5*c4*c23;
    s4s5 = s4*s5;
    c5c23 = c5*c23;
    s23c5 = s23*c5;
    s23c1 = s23*c1;
    s1s4 = s1*s4;
    s4s23c1 = s4*s23c1;
    c1c5c23 = c1*c5c23;
    s1s4s23 = s1s4*s23;
    s1c5c23 = s1*c5c23;
    s23c1c4 = s23c1*c4;
    s5s23c4 = s5s23*c4;
    
    l3s2 = l3*s2;
    l4s23 = l4*s23;
    l5c23 = l5*c23;
    l5s23 = l5*s23;
    l4c23 = l4*c23;
    l3c2 = l3*c2;
    
    A1 = (s1s23c4 + s4c1);
    A2 = (l2 - l3s2 - l4s23 + l5c23);
    A3 = (l3c2 + l4c23 + l5s23 + l6*s5c4c23 + l6*s23c5);
    A4 = (l4c23 + l5s23 + l6*s5c4c23 + l6*s23c5);
    A5 = (s1s4 - s23c1c4);
    A6 = s1c4 + s4s23c1;
    A7 = s1s4s23 - c1*c4;
    A8 = (s5s23c4 - c5c23);
    
    M = [l6*(A1*s5 - s1c5c23) - A2*s1,                        -A3*c1,                 -A4*c1,              l6*(A6)*s5,   l6*(A5*c5 - s5*c1c23),                               0;
         l6*(A5*s5 + c1c5c23) + A2*c1,                        -A3*s1,                 -A4*s1,              l6*(A7)*s5, -l6*(A1*c5 + s1*s5*c23),                               0;
                                    0, -l3s2 - l4s23 + l5c23 - l6*A8, -l4s23 + l5c23 - l6*A8,            -l6*s4s5*c23,  l6*(-s5s23 + c4*c5c23),                               0;
                                    0,                            s1,                     s1,                   c1c23,                      A6, s1*s4s5 - s5s23*c1*c4 + c1c5c23;
                                    0,                           -c1,                    -c1,                  s1*c23,                      A7, -s1*s5s23c4 + s1c5c23 - s4s5*c1;
                                    1,                             0,                      0,                     s23,                 -s4*c23,                 s5c4c23 + s23c5];
    
    
end