function M = hscurve(r0, rf, cv, ca, t0, tf)
    % Cálculo de las fases temporales
    n = int32(100*(tf-t0));
    t7 = tf-t0;
    t = linspace(0,t7,n).';

    t3 = t7*(1-1/(cv/100+1));
    t1 = t3*(1-1/(ca/100+1));
    t2 = t3-t1;
    t4 = t7-t3;
    t5 = t4+t1;
    t6 = t7-t1;

    t01 = t(t<=t1);
    t12 = t((t>t1)&(t<=t2));
    t23 = t((t>t2)&(t<=t3));
    t34 = t((t>t3)&(t<=t4));
    t45 = t((t>t4)&(t<=t5));
    t56 = t((t>t5)&(t<=t6));
    t67 = t(t>t6);
    

    % Cálculo de la velocidad constante
    Vmin = (rf-r0)/t7;
    V = Vmin*(1+cv/100);

    % Cálculo de la aceleración constante
    Amin = V/t3;
    A = Amin*(1+ca/100);
    
    % Cálculo de Jerk y fase
    J = 2*A/t1;
    w = 2*pi/t1;

    % Creación de los vectores
    m = length(r0);
    r = zeros(n,m);
    v = zeros(n,m);
    a = zeros(n,m);
    j = zeros(n,m);
    
    for i = 1:m
        r1 = r0(i)+(J(i)/2)*(t01.^3/6-t01/w^2+sin(w*t01)/w^3);
        r2 = r0(i)+(A(i)/2)*(t12.^2-t1*t12+t1^2/3-2/w^2);
        r3 = r0(i)+A(i)*(t23.^2/2-t1*t23/2+t1^2/6-1/w^2)-(J(i)/2)*((t23-t2).^3/6+sin(w*(t23-t2))/w^3-(t23-t2)/w^2);
        r4 = r0(i)+V(i)*(t34-t3/2);
        r5 = r0(i)+V(i)*(t45-t3/2)-(J(i)/2)*((t45-t4).^3/6+sin(w*(t45-t4))/w^3-(t45-t4)/w^2);
        r6 = r0(i)+V(i)*(t56-t3/2)-A(i)*((t56-t5).^2/2+t1*(t56-t5)/2+t1^2/6-1/w^2);
        r7 = rf(i)+V(i)*(t67-t7)-A(i)*((t67.^2-t7^2)/2+(t1/2-t5)*(t67-t7))+(J(i)/2)*((t67-t6).^3/6+sin(w*(t67-t6))/w^3-(t67-t7)/w^2-t1^3/6);
        r(:,i) = [r1;r2;r3;r4;r5;r6;r7];

        v1 = (J(i)/2)*(t01.^2/2+(cos(w*t01)-1)/w^2);
        v2 = A(i)*(t12-t1/2);
        v3 = A(i)*(t23-t1/2)-(J(i)/2)*((t23-t2).^2/2+(cos(w*(t23-t2))-1)/w^2);
        v4 = V(i)*ones(size(t34));
        v5 = V(i)-(J(i)/2)*((t45-t4).^2/2+(cos(w*(t45-t4))-1)/w^2);
        v6 = V(i)-A(i)*(t56+t1/2 -t5);
        v7 = V(i)-A(i)*(t67+t1/2-t5)+(J(i)/2)*((t67-t6).^2/2+(cos(w*(t67-t6))-1)/w^2);
        v(:,i) = [v1;v2;v3;v4;v5;v6;v7];

        a1 = (J(i)/2)*(t01-(sin(w*t01)/w));
        a2 = A(i)*ones(size(t12));
        a3 = A(i)-(J(i)/2)*(t23-t2 - sin(w*(t23-t2))/w);
        a4 = zeros(size(t34));
        a5 = -(J(i)/2)*(t45-t4 - sin(w*(t45-t4))/w);
        a6 = -A(i)*ones(size(t56));
        a7 = (J(i)/2)*(t67-t6-sin(w*(t67-t6))/w)-A(i);
        a(:,i) = [a1;a2;a3;a4;a5;a6;a7];

        j1 = (J(i)/2)*(1-cos(w*t01));
        j2 = zeros(size(t12));
        j3 = -(J(i)/2)*(1-cos(w*(t23-t2)));
        j4 = zeros(size(t34));
        j5 = -(J(i)/2)*(1-cos(w*(t45-t4)));
        j6 = zeros(size(t56));
        j7 = (J(i)/2)*(1-cos(w*(t67-t6)));

        j(:,i) = [j1;j2;j3;j4;j5;j6;j7];

    end
   
    M = [t+t0,r,v,a,j];
end