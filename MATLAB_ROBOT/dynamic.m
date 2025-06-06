function tau = dynamic(q,qp,qpp,f0,mu0)
% Cálculos iniciales
A06 = dkine(q);         % Localización del efector final
R06 = A06(1:3,1:3);     % Rotación del efector final
f6 = R06.'*f0.';        % Fuerza externa desde el Sistema 6 
mu6 = R06.'*mu0.';      % Fuerza externa desde el Sistema 6

n = length(q);
F = zeros(3,n+1); F(:,n+1) = f6;
M = zeros(3,n+1); M(:,n+1) = mu6;
tau = zeros(1,n);

% Longitud de cada eslabón [mm]
l1 = 345*1e-3;
l2 = 20*1e-3;
l3 = 260*1e-3;
l4 = 20*1e-3;
l5 = 260*1e-3;
l6 = 75*1e-3;

% Masas de cada eslabón [kg]
m1 = 3427.24*1e-3;
m2 = 3908.47*1e-3;
m3 = 1692.61*1e-3;
m4 = 1094.99*1e-3;
m5 = 584.34*1e-3;
m6 = 44.96*1e-3;
m = [m1,m2,m3,m4,m5,m6];

% Centros de masa de cada eslabón [m]
c1 = [-23.47, -91.09, -1.47].'*1e-3;
c2 = [-146.74, 15.22, 0.93].'*1e-3;
c3 = [7.86, -0.23, 26.73].'*1e-3;
c4 = [1.54, 75.66, 1.89].'*1e-3;
c5 = [0, -0.22, 4.71].'*1e-3;
c6 = [8.65, 8.66, 20.21].'*1e-3;
rc = [c1,c2,c3,c4,c5,c6];

% Tensores de Inercia de cada eslabón [kg.m^2]

I1 = [20774992.55,   782213.54,   -76625.41;
        782213.54, 16713894.81,  -401328.67;
        -76625.41,  -401328.67, 14710690.10]*1e-9;

I2 = [14345616.12,  1191409.78,  -369712.70;
       1191409.78, 40706297.92,   -61665.62;
       -369712.70,   -61665.62, 32996598.08]*1e-9;

I3 = [4916611.04,    1900.19,  540432.93;
         1900.19, 5175699.12,   11247.45;
       540432.93,   11247.45, 2903510.12]*1e-9;

I4 = [3668407.55,   29351.57,  -47406.95;
        29351.57, 2286831.45,  262071.47;
       -47406.95,  262071.47, 2598218.60]*1e-9;

I5 = [784503.44,     44.77,     55.00;
          44.77, 718574.07,  -6197.54;
          55.00,  -6197.54, 523706.32]*1e-9;

I6 = [70441.69, -3369.48,  1869.53;
      -3369.48, 70445.84,  1862.80;
       1869.53,  1862.80, 51568.25]*1e-9;

I = zeros(3,3,6);
I(:,:,1) = I1;
I(:,:,2) = I2;
I(:,:,3) = I3;
I(:,:,4) = I4;
I(:,:,5) = I5;
I(:,:,6) = I6;

% Gravedad [m/s^2]
g0 = [0, 0, -9.81];

% Parámetros Denavit-Hartenberg
theta = [q(1); q(2)+pi/2; q(3);  q(4); q(5); q(6)];
d     = [  l1;         0;    0;    l5;    0;   l6];
a     = [  l2;        l3;   l4;     0;    0;    0];
alpha = [pi/2;         0; pi/2; -pi/2; pi/2;    0];

R = zeros(3,3,n+1);   R(:,:,n+1) = eye(3);
r = zeros(3,n);     ac = zeros(3,n);
w = zeros(3,n+1);   wp = zeros(3,n+1);
vp = zeros(3,n+1);    vp(:,1) = -g0;
z0 = [0,0,1].';

% Recursión hacia adelante
for i = 1:n
    A = mdh(theta(i),d(i),a(i),alpha(i));
    R(:,:,i) = A(1:3,1:3);
    r(:,i) = [a(i), d(i)*sin(alpha(i)), d(i)*cos(alpha(i))].';
    w(:,i+1) = R(:,:,i).'*(w(:,i)+qp(i)*z0);
    wp(:,i+1) = R(:,:,i).'*(wp(:,i)+qpp(i)*z0) + cross(w(:,i),[0,0,qp(i)].');
    vp(:,i+1) = R(:,:,i).'*vp(:,i)+cross(wp(:,i+1),r(:,i))+cross(w(:,i+1),cross(w(:,i+1),r(:,i)));  
end

for i = 1:n
    ac(:,i) = vp(:,i+1) + cross(wp(:,i+1),rc(:,i)) + cross(w(:,i+1),cross(w(:,i+1),rc(:,i)));
end

% Recursión hacia atras
for i = n:-1:1
    F(:,i) = R(:,:,i+1)*F(:,i+1)+m(i)*ac(:,i);
    M(:,i) = R(:,:,i+1)*(M(:,i+1)+cross(R(:,:,i+1).'*r(:,i),F(:,i+1)))+cross(r(:,i)+rc(:,i),m(i)*ac(:,i))+I(:,:,i)*wp(:,i+1)+cross(w(:,i+1),I(:,:,i)*w(:,i+1));
    tau(i) = M(:,i).'*R(:,:,i).'*z0;
end

end