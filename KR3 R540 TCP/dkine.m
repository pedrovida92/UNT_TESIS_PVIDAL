function T = dkine(q)
    l1 = 345;
    l2 = 20;
    l3 = 260;
    l4 = 20;
    l5 = 260;
    l6 = 75+25;
    
    theta = [q(1); q(2)+pi/2; q(3);  q(4); q(5); q(6)];
    d     = [  l1;         0;    0;    l5;    0;   l6];
    a     = [  l2;        l3;   l4;     0;    0;    0];
    alpha = [pi/2;         0; pi/2; -pi/2; pi/2;    0];
    
    A01 = mdh(theta(1),d(1),a(1),alpha(1));
    A12 = mdh(theta(2),d(2),a(2),alpha(2));
    A23 = mdh(theta(3),d(3),a(3),alpha(3));
    A34 = mdh(theta(4),d(4),a(4),alpha(4));
    A45 = mdh(theta(5),d(5),a(5),alpha(5));
    A56 = mdh(theta(6),d(6),a(6),alpha(6));

    T = A01*A12*A23*A34*A45*A56;
    T = evalz(T);
end