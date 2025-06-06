function M = scurve(r0, rf, c, t0, tf)
    % Cálculo de las fases temporales
    n = int32(100*(tf-t0));
    t3 = tf-t0;
    t = linspace(0,t3,n).';
    t1 = t3*(1-1/(c/100+1));
    t2 = t3-t1;
    t01 = t(t<=t1);
    t12 = t((t>t1)&(t<=t2));
    t23 = t(t>t2);

    % Cálculo de la velocidad constante
    v_min = (rf-r0)/t3;
    v_max = 2*v_min;
    vc = v_min + (c/100)*(v_max-v_min);
    
    % Cálculo de constantes
    tj = t1;
    J = 2*pi*vc/(tj^2);
    w = 2*pi/tj;

    % Creación de los vectores
    m = length(r0);
    r = zeros(n,m);
    v = zeros(n,m);
    a = zeros(n,m);
    
    for i = 1:m
        r1 = r0(i) + (J(i)/w)*(t01.^2/2 + (cos(w*t01)-1)/w^2);
        r2 = r0(i) + vc(i)*(t12-t1/2);
        r3 = rf(i) + vc(i)*(t23-t3)-(J(i)/w)*((t23.^2 - t3^2)/2 - t2*(t23-t3) + (cos(w*(t23-t2))-1)/w^2);
        r(:,i) = [r1;r2;r3];

        v1 = (J(i)/w)*(t01-sin(w*t01)/w);
        v2 = vc(i)*ones(size(t12));
        v3 = vc(i) - (J(i)/w)*(t23-t2-sin(w*(t23-t2))/w);
        v(:,i) = [v1;v2;v3];

        a1 = (J(i)/w)*(1-cos(w*t01));
        a2 = zeros(size(t12));
        a3 = -(J(i)/w)*(1-cos(w*(t23-t2)));
        a(:,i) = [a1;a2;a3];
    end
   
    M = [t+t0,r,v,a];
end